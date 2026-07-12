#!/usr/bin/env python3
# Copyright 2026, Beyley Cardellio
# SPDX-License-Identifier: BSL-1.0

"""Generate forwarding macros for versioned OpenVR interface implementations."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys


CLASS_PATTERN = re.compile(r"^\s*class\s+(?P<class_name>[A-Za-z_]\w*)\b")
VERSIONED_CLASS_PATTERN = re.compile(r"^(?P<family>[A-Za-z_]\w*)_(?P<version>[0-9][0-9A-Za-z]*)$")
METHOD_PATTERN = re.compile(
    r"^virtual\s+(?P<return_type>.*?)\s*(?P<method_name>~?[A-Za-z_]\w*)\s*\((?P<params>.*)\)\s*(?P<qualifiers>[^;=]*)=\s*0\s*;$"
)


@dataclass(frozen=True)
class MethodDefinition:
    family: str
    version: str
    class_name: str
    return_type: str
    method_name: str
    params_text: str
    qualifiers: str
    call_args: tuple[str, ...]


def label_sort_key(label: str) -> tuple[tuple[int | str, ...], str]:
    parts = re.findall(r"\d+|[A-Za-z]+", label)
    key = tuple(int(part) if part.isdigit() else part.lower() for part in parts)
    return key, label.lower()


def versioned_classes(header_text: str) -> list[tuple[str, str, list[str]]]:
    lines = header_text.splitlines()
    classes: list[tuple[str, str, list[str]]] = []

    current_class_name: str | None = None
    current_family: str | None = None
    current_lines: list[str] = []
    in_class = False

    for line in lines:
        if not in_class:
            match = CLASS_PATTERN.match(line)
            if match is None:
                continue

            class_name = match.group("class_name")
            family_match = VERSIONED_CLASS_PATTERN.match(class_name)
            if family_match is None:
                continue

            current_class_name = class_name
            current_family = family_match.group("family")
            current_lines = [line]
            in_class = True
            continue

        current_lines.append(line)
        if line.strip() == "};":
            assert current_class_name is not None
            assert current_family is not None
            classes.append((current_family, current_class_name, current_lines))
            current_class_name = None
            current_family = None
            current_lines = []
            in_class = False

    return classes


def split_top_level(text: str, delimiter: str) -> list[str]:
    items: list[str] = []
    current: list[str] = []
    angle_depth = 0
    paren_depth = 0
    bracket_depth = 0
    brace_depth = 0

    for char in text:
        if char == "<":
            angle_depth += 1
        elif char == ">" and angle_depth > 0:
            angle_depth -= 1
        elif char == "(":
            paren_depth += 1
        elif char == ")" and paren_depth > 0:
            paren_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]" and bracket_depth > 0:
            bracket_depth -= 1
        elif char == "{":
            brace_depth += 1
        elif char == "}" and brace_depth > 0:
            brace_depth -= 1

        if (
            char == delimiter
            and angle_depth == 0
            and paren_depth == 0
            and bracket_depth == 0
            and brace_depth == 0
        ):
            items.append("".join(current).strip())
            current = []
            continue

        current.append(char)

    if current:
        items.append("".join(current).strip())

    return items


def strip_default_value(param_text: str) -> str:
    pieces = split_top_level(param_text, "=")
    return pieces[0].strip() if pieces else param_text.strip()


def extract_param_name(param_text: str) -> str | None:
    stripped = strip_default_value(param_text)
    if not stripped or stripped == "void":
        return None

    identifiers = re.findall(r"[A-Za-z_]\w*", stripped)
    if not identifiers:
        return None

    ignored = {
        "const",
        "volatile",
        "struct",
        "class",
        "enum",
        "signed",
        "unsigned",
        "short",
        "long",
        "void",
        "bool",
        "char",
        "int",
        "float",
        "double",
        "auto",
        "typename",
        "decltype",
    }
    for identifier in reversed(identifiers):
        if identifier not in ignored:
            return identifier

    return identifiers[-1]


def parse_method(method_text: str, family: str, version: str, class_name: str) -> MethodDefinition | None:
    match = METHOD_PATTERN.match(method_text)
    if match is None:
        return None

    method_name = match.group("method_name")
    if method_name.startswith("~"):
        return None

    params_text = match.group("params").strip()
    params = [] if not params_text else split_top_level(params_text, ",")
    call_args = tuple(
        name
        for name in (extract_param_name(param) for param in params)
        if name is not None
    )

    return MethodDefinition(
        family=family,
        version=version,
        class_name=class_name,
        return_type=match.group("return_type").strip(),
        method_name=method_name,
        params_text=params_text,
        qualifiers=match.group("qualifiers").strip(),
        call_args=call_args,
    )


def class_methods(family: str, version: str, class_name: str, class_lines: list[str]) -> list[MethodDefinition]:
    methods: list[MethodDefinition] = []
    current: list[str] = []
    collecting = False

    for line in class_lines:
        stripped = line.strip()
        if not collecting and stripped.startswith("virtual "):
            collecting = True
            current = [stripped]
        elif collecting:
            current.append(stripped)

        if collecting and "= 0;" in stripped:
            method = parse_method(" ".join(current), family, version, class_name)
            if method is not None:
                methods.append(method)
            collecting = False
            current = []

    return methods


def collect_methods(header_text: str) -> tuple[dict[str, str], dict[str, list[MethodDefinition]]]:
    latest_class_by_family: dict[str, str] = {}
    methods_by_family: dict[str, dict[tuple[str, str, str, str], MethodDefinition]] = {}

    for family, class_name, class_lines in versioned_classes(header_text):
        family_match = VERSIONED_CLASS_PATTERN.match(class_name)
        assert family_match is not None
        version = family_match.group("version")

        previous_class = latest_class_by_family.get(family)
        if previous_class is None:
            latest_class_by_family[family] = class_name
        else:
            previous_version = VERSIONED_CLASS_PATTERN.match(previous_class).group("version")
            if label_sort_key(version) > label_sort_key(previous_version):
                latest_class_by_family[family] = class_name

        family_methods = methods_by_family.setdefault(family, {})
        for method in class_methods(family, version, class_name, class_lines):
            key = (
                method.method_name,
                method.return_type,
                method.params_text,
                method.qualifiers,
            )
            family_methods[key] = method

    normalized_methods = {
        family: sorted(
            unique_methods.values(),
            key=lambda method: (
                method.method_name.lower(),
                method.params_text,
                method.return_type,
                method.qualifiers,
            ),
        )
        for family, unique_methods in methods_by_family.items()
    }
    return latest_class_by_family, normalized_methods


def macro_name(method: MethodDefinition, overload_suffix: str | None) -> str:
    if overload_suffix is None:
        return f"Forward_{method.family}_{method.method_name}"
    return f"Forward_{method.family}_{method.method_name}_{overload_suffix}"


def declaration_with_override(method: MethodDefinition) -> str:
    qualifier_suffix = f" {method.qualifiers}" if method.qualifiers else ""
    return (
        f"virtual {method.return_type} {method.method_name}( {method.params_text} )"
        f"{qualifier_suffix} override"
    )


def forward_body(method: MethodDefinition) -> str:
    call_args = ", ".join(method.call_args)
    qualifier_suffix = f" {method.qualifiers}" if method.qualifiers else ""
    call = f"ForwardBase_{method.family}::{method.method_name}( {call_args} )"
    if method.return_type == "void":
        return (
            f"virtual {method.return_type} {method.method_name}( {method.params_text} ){qualifier_suffix} override {{ \\\n"
            f"        {call}; \\\n"
            f"    }}"
        )

    return (
        f"virtual {method.return_type} {method.method_name}( {method.params_text} ){qualifier_suffix} override {{ \\\n"
        f"        return {call}; \\\n"
        f"    }}"
    )


def generate_header(header_name: str, latest_class_by_family: dict[str, str], methods_by_family: dict[str, list[MethodDefinition]]) -> str:
    lines = [
        "//========= Copyright Valve Corporation ============//",
        "",
        "#pragma once",
        "",
        f'#include "{header_name}"',
        "",
        "// Generated forwarding macros for implementing older OpenVR interfaces on top of a newer concrete class.",
        "// Usage pattern:",
        "//   ForwardDeclareBase_IVRSystem(XRTVRSystem_026)",
        "//   Forward_IVRSystem_GetRecommendedRenderTargetSize()",
        "",
    ]

    for family in sorted(methods_by_family):
        lines.extend(
            (
                f"// {family} forwards to a caller-provided base, typically {latest_class_by_family[family]}.",
                f"#define ForwardDeclareBase_{family}(BaseType) using ForwardBase_{family} = BaseType",
                "",
            )
        )

        methods = methods_by_family[family]
        overload_counts: dict[str, int] = {}
        overload_version_counts: dict[str, dict[str, int]] = {}
        for method in methods:
            overload_counts[method.method_name] = overload_counts.get(method.method_name, 0) + 1
            version_counts = overload_version_counts.setdefault(method.method_name, {})
            version_counts[method.version] = version_counts.get(method.version, 0) + 1

        overload_version_positions: dict[str, dict[str, int]] = {}
        for method in methods:
            suffix = None
            if overload_counts[method.method_name] > 1:
                suffix = method.version
                if overload_version_counts[method.method_name][method.version] > 1:
                    version_positions = overload_version_positions.setdefault(method.method_name, {})
                    version_positions[method.version] = version_positions.get(method.version, 0) + 1
                    suffix = f"{method.version}_{version_positions[method.version]}"

            macro = macro_name(method, suffix)
            lines.append(
                f"// {method.class_name}::{method.method_name}"
            )
            lines.append(f"#define {macro}() \\")
            lines.append(f"    {forward_body(method)}")
            lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate forwarding macros from the unified OpenVR interface header."
    )
    parser.add_argument(
        "-i",
        "--input-file",
        type=Path,
        required=True,
        help="Unified interface header to read.",
    )
    parser.add_argument(
        "-o",
        "--output-file",
        type=Path,
        required=True,
        help="Header file to write.",
    )
    parser.add_argument(
        "--include",
        required=True,
        help="Header to include at the top of the generated file.",
    )
    args = parser.parse_args()

    input_file = args.input_file.resolve()
    if not input_file.exists():
        parser.error(f"{input_file} does not exist")

    output_file = args.output_file.resolve()
    output_file.parent.mkdir(parents=True, exist_ok=True)

    header_text = input_file.read_text(encoding="utf-8")
    latest_class_by_family, methods_by_family = collect_methods(header_text)
    if not methods_by_family:
        print(f"No versioned interface methods found in {input_file}", file=sys.stderr)
        return 1

    include_name = args.include
    output_file.write_text(
        generate_header(include_name, latest_class_by_family, methods_by_family),
        encoding="utf-8",
    )
    macro_count = sum(len(methods) for methods in methods_by_family.values())
    print(f"Wrote {macro_count} forwarding macro(s) to {output_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
