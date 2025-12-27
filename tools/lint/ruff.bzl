"""Ruff module extension for linting Python code."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# buildifier: disable=unused-variable
def _ruff_ext_impl(ctx):
    ruff_version = "0.14.0"
    http_archive(
        name = "ruff-linux",
        build_file_content = 'exports_files(["ruff"])',
        strip_prefix = "ruff-aarch64-unknown-linux-gnu",
        sha256 = "60fc3b464a723456ba1bc8cd91d29806753c885f72e26ac67b718cee6b35aaca",
        urls = [
            "https://github.com/astral-sh/ruff/releases/download/{0}/ruff-aarch64-unknown-linux-gnu.tar.gz".format(ruff_version),
        ],
    )

ruff = module_extension(implementation = _ruff_ext_impl)
