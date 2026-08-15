"""MechForge packaging script: fetch embedded Python runtimes per version+platform
and extract them into build/<target>/python/.

Usage:
    python3 scripts/package.py --target <target> [--target <target> ...]

    Examples:
        python3 scripts/package.py --target linux-x86_64              # download + extract
        python3 scripts/package.py --target linux-x86_64 --target windows-x86_64
        python3 scripts/package.py --target linux-x86_64 --print-urls # print URLs only
        python3 scripts/package.py --target linux-x86_64 --no-extract # download only
        python3 scripts/package.py --target linux-x86_64 --python-version 3.14.7

    <target> choices:
        linux-x86_64 | linux-aarch64 | linux-musl-x86_64 | linux-musl-aarch64
        | macos-x86_64 | macos-aarch64 | windows-x86_64

    Archives are cached in downloads/ (skipped if already present and sha256
    matches); extracted runtimes land in build/<target>/python/.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import tarfile
import time
from http.client import IncompleteRead
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

PYTHON_VERSION = "3.14.7"

LATEST_RELEASE_JSON = "https://raw.githubusercontent.com/astral-sh/python-build-standalone/latest-release/latest-release.json"
# the response is a JSON object like:
# {
#   "version": 1,
#   "tag": "20260814",
#   "release_url": "https://github.com/astral-sh/python-build-standalone/releases/tag/20260814",
#   "asset_url_prefix": "https://github.com/astral-sh/python-build-standalone/releases/download/20260814"
# }

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DOWNLOADS_DIR = PROJECT_ROOT / "downloads"
BUILD_DIR = PROJECT_ROOT / "build"

# Map packaging targets to python-build-standalone platform triples.
TARGET_TRIPLES: dict[str, str] = {
    "linux-x86_64": "x86_64-unknown-linux-gnu",
    "linux-aarch64": "aarch64-unknown-linux-gnu",
    "linux-musl-x86_64": "x86_64-unknown-linux-musl",
    "linux-musl-aarch64": "aarch64-unknown-linux-musl",
    "macos-x86_64": "x86_64-apple-darwin",
    "macos-aarch64": "aarch64-apple-darwin",
    "windows-x86_64": "x86_64-pc-windows-msvc",
}

# Asset flavor preference: stripped first (≈35 MB vs ≈124 MB, no debug symbols),
# then the plain install_only archive. Both keep pip + site-packages + headers.
INSTALL_ONLY_FLAVORS = ("install_only_stripped", "install_only")

MAX_RETRIES = 6
RETRY_BASE_DELAY = 2.0

# Module-level caches so a multi-target run only fetches these once.
_LATEST_META_CACHE: dict[str, object] | None = None
_SUMS_CACHE: dict[str, str] | None = None


# ------------------------------------------------------------------- metadata


def _fetch_bytes(url: str, *, retries: int = MAX_RETRIES) -> bytes:
    """Fetch a small resource with retry/backoff against transient errors."""
    last_exc: BaseException | None = None
    for attempt in range(retries):
        try:
            req = Request(url, headers={"User-Agent": "mechforge-package-script"})
            with urlopen(req, timeout=60) as resp:
                return resp.read()
        except HTTPError as exc:
            if exc.code >= 500 or exc.code == 429:
                last_exc = exc
            else:
                raise
        except (URLError, IncompleteRead, TimeoutError, OSError) as exc:
            last_exc = exc
        if attempt < retries - 1:
            time.sleep(RETRY_BASE_DELAY * (2**attempt))
    assert last_exc is not None
    raise last_exc


def _latest_release_meta() -> dict[str, object]:
    global _LATEST_META_CACHE
    if _LATEST_META_CACHE is None:
        _LATEST_META_CACHE = json.loads(
            _fetch_bytes(LATEST_RELEASE_JSON).decode("utf-8")
        )
    assert _LATEST_META_CACHE is not None
    return _LATEST_META_CACHE


def _latest_tag() -> str:
    return str(_latest_release_meta()["tag"])


def _asset_url_prefix() -> str:
    prefix = _latest_release_meta().get("asset_url_prefix")
    if not isinstance(prefix, str):
        raise RuntimeError("latest-release.json missing 'asset_url_prefix'")
    return prefix


def _fetch_sha256_sums() -> dict[str, str]:
    """Fetch the release SHA256SUMS manifest -> {filename: sha256hex}."""
    global _SUMS_CACHE
    if _SUMS_CACHE is None:
        text = _fetch_bytes(f"{_asset_url_prefix()}/SHA256SUMS").decode("utf-8")
        # Example lines from SHA256SUMS:
        # 5cb723791b969d1584eb2efdb8d36ec80ff33b4b47134007859b9a9f85a02dff  cpython-3.10.21+20260814-aarch64-apple-darwin-debug-full.tar.zst
        # 935e7112b2a567388d2f9d99eec2eb9957a7b6098248f10890073dc870537514  cpython-3.10.21+20260814-aarch64-apple-darwin-install_only.tar.gz

        sums: dict[str, str] = {}
        for line in text.splitlines():
            parts = line.split()
            if len(parts) >= 2 and re.fullmatch(r"[0-9a-f]{64}", parts[0]):
                sums[parts[1]] = parts[0]
        _SUMS_CACHE = sums

    return _SUMS_CACHE


def _resolve_asset(
    python_version: str, target: str, sums: dict[str, str]
) -> tuple[str, str, str]:
    """Return (flavor, filename, sha256) for the best asset of this target."""
    triple = TARGET_TRIPLES[target]
    tag = _latest_tag()
    for flavor in INSTALL_ONLY_FLAVORS:
        name = f"cpython-{python_version}+{tag}-{triple}-{flavor}.tar.gz"
        sha = sums.get(name)
        if sha:
            return flavor, name, sha
    raise RuntimeError(
        f"No matching python-build-standalone asset for Python {python_version} "
        f"target '{target}' (triple {triple}) in release {tag}"
    )


def populate_python_urls(python_version: str) -> dict[str, str]:
    sums = _fetch_sha256_sums()
    prefix = _asset_url_prefix()
    return {
        target: f"{prefix}/{quote(_resolve_asset(python_version, target, sums)[1])}"
        for target in TARGET_TRIPLES
    }


# ------------------------------------------------------------------ downloading


def _verify_sha256(path: Path, expected: str) -> bool:
    if not path.exists():
        return False
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest() == expected


def _download_once(url: str, tmp: Path, offset: int) -> None:
    headers = {"User-Agent": "mechforge-package-script", "Accept": "*/*"}
    if offset:
        headers["Range"] = f"bytes={offset}-"
    req = Request(url, headers=headers)
    with urlopen(req, timeout=120) as resp:
        if offset and resp.status != 206:
            # 206 means the server accepted our Range request and is sending the remaining bytes.
            offset = 0  # server ignored Range -> full body, restart
        mode = "ab" if offset else "wb"
        with tmp.open(mode) as fh:
            while True:
                chunk = resp.read(1 << 20)
                if not chunk:
                    break
                fh.write(chunk)


def _download_with_retry(url: str, dest: Path, sha256_hex: str) -> None:
    DOWNLOADS_DIR.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    last_exc: BaseException | None = None
    for attempt in range(MAX_RETRIES):
        try:
            offset = tmp.stat().st_size if tmp.exists() else 0
            # The offset of the file pointer,
            # which is the current size of the temporary file.
            # It is used to resume the download from where it left off in case of a retry.
            _download_once(url, tmp, offset)
            break
        except HTTPError as exc:
            if exc.code >= 500 or exc.code == 429:
                # Retry on server errors (5xx) or too many requests (429)
                last_exc = exc
            else:
                raise
        except (URLError, IncompleteRead, TimeoutError, OSError) as exc:
            last_exc = exc
        if attempt < MAX_RETRIES - 1:
            time.sleep(RETRY_BASE_DELAY * (2**attempt))
    else:
        assert last_exc is not None
        raise last_exc

    if not _verify_sha256(tmp, sha256_hex):
        tmp.unlink(missing_ok=True)
        raise RuntimeError(f"SHA256 mismatch for {dest.name} (expected {sha256_hex})")
    tmp.replace(dest)


def ensure_python_archive(python_version: str, target: str) -> Path:
    """Download (or reuse the cached) python archive into downloads/."""
    if target not in TARGET_TRIPLES:
        known = ", ".join(sorted(TARGET_TRIPLES))
        raise ValueError(f"Unknown target '{target}'. Known targets: {known}")

    sums = _fetch_sha256_sums()
    _flavor, name, sha = _resolve_asset(python_version, target, sums)
    dest = DOWNLOADS_DIR / name

    if dest.exists() and _verify_sha256(dest, sha):
        print(f"[cache hit] {target}: {dest}")
        return dest

    url = f"{_asset_url_prefix()}/{quote(name)}"
    print(f"[download] {target}: {url}")
    _download_with_retry(url, dest, sha)
    print(f"[saved] {dest}  (sha256 ok)")
    return dest


def install_python_runtime(python_version: str, target: str) -> Path:
    """Extract the archive into build/<target>/python/ (idempotent)."""
    archive = ensure_python_archive(python_version, target)
    target_dir = BUILD_DIR / target
    python_dir = target_dir / "python"

    if (python_dir / "bin").exists() or (python_dir / "python.exe").exists():
        print(f"[runtime ok] {target}: {python_dir}")
        return python_dir

    target_dir.mkdir(parents=True, exist_ok=True)
    if python_dir.exists():
        shutil.rmtree(python_dir)

    print(f"[extract] {target}: {archive.name} -> {target_dir}")
    with tarfile.open(archive, "r:gz") as tf:
        tf.extractall(target_dir, filter="data")

    print(f"[runtime ready] {target}: {python_dir}")
    return python_dir


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Resolve python-build-standalone archives by Python version and target: "
            "download to downloads/ (cached), then extract to build/<target>/python/."
        )
    )
    parser.add_argument(
        "--python-version",
        default=PYTHON_VERSION,
        help=f"CPython version (default: {PYTHON_VERSION})",
    )
    parser.add_argument(
        "--target",
        action="append",
        required=True,
        choices=sorted(TARGET_TRIPLES),
        help="Packaging target (repeat to fetch multiple)",
    )
    parser.add_argument(
        "--print-urls",
        action="store_true",
        help="Only print resolved asset URLs for this version (no download)",
    )
    parser.add_argument(
        "--no-extract",
        action="store_true",
        help="Download to downloads/ but do not extract to build/<target>/",
    )
    return parser.parse_args(argv)


def package(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    python_version: str = args.python_version
    targets: list[str] = args.target

    if args.print_urls:
        url_map = populate_python_urls(python_version)
        out = {target: url_map[target] for target in targets}
        print(json.dumps(out, indent=2, ensure_ascii=True))
        return 0

    for target in targets:
        if args.no_extract:
            ensure_python_archive(python_version, target)
        else:
            install_python_runtime(python_version, target)

    return 0


if __name__ == "__main__":
    raise SystemExit(package())
