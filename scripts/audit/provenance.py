#!/usr/bin/env python3
import argparse
import hashlib
import json
import pathlib


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"sha256": digest.hexdigest(), "size": path.stat().st_size}


def parse_revision(west_text, project):
    for line in west_text.splitlines():
        fields = line.split()
        if project in fields:
            index = fields.index(project)
            if index > 0:
                return fields[index - 1]
    return "unknown"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--text", required=True)
    parser.add_argument("--west", required=True)
    parser.add_argument("--artifacts", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--image-digest", required=True)
    parser.add_argument("--mode", required=True)
    parser.add_argument("--board", required=True)
    parser.add_argument("--keymap", required=True)
    parser.add_argument("--snippet", required=True)
    args = parser.parse_args()

    source = pathlib.Path(args.source)
    artifacts = pathlib.Path(args.artifacts)
    west_path = pathlib.Path(args.west)
    metadata = source / "audit-source.json"
    source_data = json.loads(metadata.read_text()) if metadata.exists() else {}
    west_text = west_path.read_text(errors="replace") if west_path.exists() else "unknown"
    shields = ["charybdis_left", "charybdis_right"] if args.mode == "both" else [f"charybdis_{args.mode}"]
    data = {
        "source_commit": source_data.get("source_commit", "unknown"),
        "source_tree": source_data.get("source_tree", "unknown"),
        "dirty": source_data.get("dirty", "unknown"),
        "source_status": source_data.get("status", "unknown"),
        "input_files": {},
        "west_list": west_text,
        "zmk_sha": parse_revision(west_text, "zmk"),
        "zephyr_sha": parse_revision(west_text, "zephyr"),
        "docker_image": args.image or "unknown",
        "docker_digest": args.image_digest or "unknown",
        "mode": args.mode,
        "board": args.board,
        "shields": shields,
        "keymap": args.keymap,
        "snippet": args.snippet,
        "cmake_args": [
            f"-DSHIELD=charybdis_{args.mode}",
            "-DZMK_CONFIG=/config",
            "-DZMK_EXTRA_MODULES=/config/vendor/pmw3610-driver",
        ],
        "artifacts": {},
    }
    for path in sorted(p for p in source.rglob("*") if p.is_file() and p.name != "audit-source.json"):
        data["input_files"][str(path.relative_to(source))] = file_hash(path)
    for path in sorted(p for p in artifacts.rglob("*") if p.is_file() and p.name not in {"provenance.json", "provenance.txt"}):
        data["artifacts"][str(path.relative_to(artifacts))] = file_hash(path)
    pathlib.Path(args.output).write_text(json.dumps(data, indent=2) + "\n")
    pathlib.Path(args.text).write_text("\n".join(f"{key}: {value}" for key, value in data.items()) + "\n")


if __name__ == "__main__":
    main()
