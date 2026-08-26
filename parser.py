"""Parser module for reading and validating Fly-in network map files."""

import re
from typing import Dict, Final, Set
from drone import Drone
from edge import Edge
from graph import Graph, GraphError
from node import Node

VALID_ZONE_TYPES: Final[Set[str]] = {"normal", "blocked", "restricted", "priority"}
ALLOWED_ZONE_META_TAGS: Final[Set[str]] = {"zone", "color", "max_drones"}
ALLOWED_CONN_META_TAGS: Final[Set[str]] = {"max_link_capacity"}


class ParsingError(Exception):
    """Exception raised when input map file violates syntax or constraints."""

    def __init__(self, line_num: int, message: str) -> None:
        """Initializes a ParsingError with line number and diagnostic message.

        Args:
            line_num: 1-indexed line number in the input file.
            message: Explanation of the parsing failure.
        """
        super().__init__(f"Line {line_num}: {message}")
        self.line_num = line_num
        self.message = message


class Parser:
    """Parses Fly-in map files into Graph instances with strict sequential validation."""

    def __init__(self) -> None:
        """Initializes a new Parser instance."""
        self._hub_prefix_pattern = re.compile(r"^(start_hub|end_hub|hub):\s*(.*)$")
        self._conn_prefix_pattern = re.compile(r"^connection:\s*(.*)$")
        self._drones_prefix_pattern = re.compile(r"^nb_drones:\s*(.*)$")

    def _strip_comments(self, line: str) -> str:
        """Strips comments starting with '#' while preserving line content.

        Args:
            line: Raw line text.

        Returns:
            Line text with comment stripped.
        """
        if "#" in line:
            line = line.split("#", 1)[0]
        return line.strip()

    def _parse_bracket_metadata(
        self,
        raw_meta: str,
        allowed_tags: Set[str],
        line_num: int
    ) -> Dict[str, str]:
        """Validates and extracts key-value pairs from a bracketed metadata string.

        Args:
            raw_meta: Raw metadata string including enclosing brackets.
            allowed_tags: Set of allowed tag keys for this entity type.
            line_num: Current line number for error reporting.

        Returns:
            Dictionary mapping metadata keys to string values.

        Raises:
            ParsingError: If bracket syntax, tag names, or values are invalid.
        """
        clean = raw_meta.strip()
        if not clean.startswith("[") or not clean.endswith("]"):
            raise ParsingError(line_num, f"Malformed metadata block '{clean}' (must be enclosed in '[...]')")

        content = clean[1:-1].strip()
        if not content:
            return {}

        metadata: Dict[str, str] = {}
        tokens = content.split()
        for token in tokens:
            if "=" not in token:
                raise ParsingError(line_num, f"Invalid metadata tag '{token}' (expected key=value)")
            key, val = token.split("=", 1)
            key = key.strip()
            val = val.strip()
            if not key or not val:
                raise ParsingError(line_num, f"Empty key or value in metadata tag '{token}'")

            if key not in allowed_tags:
                allowed_str = ", ".join(sorted(allowed_tags))
                raise ParsingError(line_num, f"Invalid metadata tag '{key}' (allowed tags: {allowed_str})")

            if key in metadata:
                raise ParsingError(line_num, f"Duplicate metadata tag '{key}' in same block")

            metadata[key] = val

        return metadata

    def parse_file(self, graph: Graph, filename: str) -> None:
        """Parses a map file strictly sequentially and populates the given Graph object.

        Args:
            graph: Graph object to populate.
            filename: Path to the map file.

        Raises:
            ParsingError: If any syntax, ordering, or semantic constraint is violated.
            FileNotFoundError: If the map file does not exist.
        """
        has_nb_drones = False
        last_line_num = 1

        with open(filename, "r", encoding="utf-8") as f:
            for line_num, raw_line in enumerate(f, 1):
                last_line_num = line_num
                clean_line = self._strip_comments(raw_line)
                if not clean_line:
                    continue

                # Check if first non-empty line is nb_drones
                if not has_nb_drones:
                    drones_match = self._drones_prefix_pattern.match(clean_line)
                    if not drones_match:
                        raise ParsingError(
                            line_num,
                            f"First declaration must be 'nb_drones: <positive_integer>', found '{clean_line}'"
                        )
                    drones_val = drones_match.group(1).strip()
                    if not drones_val.isdigit() or int(drones_val) < 1:
                        raise ParsingError(
                            line_num,
                            f"nb_drones must be a positive integer (> 0), got '{drones_val}'"
                        )
                    num_drones = int(drones_val)
                    graph.drones = [Drone(i) for i in range(num_drones)]
                    has_nb_drones = True
                    continue

                # Disallow duplicate nb_drones
                if self._drones_prefix_pattern.match(clean_line):
                    raise ParsingError(line_num, "Duplicate 'nb_drones' declaration is forbidden")

                # Parse hub line: start_hub, end_hub, hub
                hub_match = self._hub_prefix_pattern.match(clean_line)
                if hub_match:
                    self._parse_hub_line(graph, hub_match.group(1), hub_match.group(2).strip(), line_num)
                    continue

                # Parse connection line: connection: <u_zone>-<v_zone> [metadata]
                conn_match = self._conn_prefix_pattern.match(clean_line)
                if conn_match:
                    self._parse_connection_line(graph, conn_match.group(1).strip(), line_num)
                    continue

                # Unrecognized line format
                raise ParsingError(line_num, f"Unrecognized or malformed line format: '{clean_line}'")

        if not has_nb_drones:
            raise ParsingError(1, "Map file is empty or missing 'nb_drones' declaration")

        # Validate that both start_hub and end_hub were defined
        try:
            graph.validate()
        except GraphError as err:
            raise ParsingError(last_line_num, str(err))

    def _parse_hub_line(self, graph: Graph, hub_type: str, remainder: str, line_num: int) -> None:
        """Parses and validates an individual zone / hub definition line.

        Args:
            graph: Target Graph instance.
            hub_type: One of 'start_hub', 'end_hub', or 'hub'.
            remainder: The string following the '<type>:' prefix.
            line_num: 1-indexed line number.

        Raises:
            ParsingError: If formatting, coordinates, names, or metadata are invalid.
        """
        # Separate metadata block if present
        meta_dict: Dict[str, str] = {}
        if "[" in remainder:
            bracket_idx = remainder.find("[")
            core_part = remainder[:bracket_idx].strip()
            meta_part = remainder[bracket_idx:].strip()
            meta_dict = self._parse_bracket_metadata(meta_part, ALLOWED_ZONE_META_TAGS, line_num)
        else:
            core_part = remainder.strip()

        parts = core_part.split()
        if len(parts) != 3:
            raise ParsingError(
                line_num,
                f"Malformed {hub_type} definition '{remainder}' (expected: <name> <x> <y> [metadata])"
            )

        name, x_str, y_str = parts[0], parts[1], parts[2]

        # Name constraints: no dashes and no spaces
        if "-" in name:
            raise ParsingError(line_num, f"Zone name '{name}' is invalid: dashes ('-') are forbidden in zone names")

        # Coordinate constraints: integer values
        try:
            x = int(x_str)
        except ValueError:
            raise ParsingError(line_num, f"Invalid X coordinate '{x_str}' (coordinates must be integers)")

        try:
            y = int(y_str)
        except ValueError:
            raise ParsingError(line_num, f"Invalid Y coordinate '{y_str}' (coordinates must be integers)")

        # Validate zone type
        zone = meta_dict.get("zone", "normal")
        if zone not in VALID_ZONE_TYPES:
            valid_str = ", ".join(sorted(VALID_ZONE_TYPES))
            raise ParsingError(line_num, f"Invalid zone type '{zone}' (must be one of: {valid_str})")

        cost = 2 if zone == "restricted" else 1
        color = meta_dict.get("color", "")

        # Validate max_drones
        max_drones = 1
        if "max_drones" in meta_dict:
            max_str = meta_dict["max_drones"]
            if not max_str.isdigit() or int(max_str) < 1:
                raise ParsingError(line_num, f"max_drones must be a positive integer (> 0), got '{max_str}'")
            max_drones = int(max_str)

        node = Node(
            name=name,
            x=x,
            y=y,
            zone=zone,
            cost=cost,
            color=color,
            max_drones=max_drones
        )

        try:
            if hub_type == "start_hub":
                graph.add_start(node)
            elif hub_type == "end_hub":
                graph.add_end(node)
            else:
                graph.add_node(node)
        except GraphError as err:
            raise ParsingError(line_num, str(err))

    def _parse_connection_line(self, graph: Graph, remainder: str, line_num: int) -> None:
        """Parses and validates a connection definition line linking two pre-existing zones.

        Args:
            graph: Target Graph instance.
            remainder: The string following the 'connection:' prefix.
            line_num: 1-indexed line number.

        Raises:
            ParsingError: If endpoints are not yet defined, syntax is malformed, or duplicate.
        """
        meta_dict: Dict[str, str] = {}
        if "[" in remainder:
            bracket_idx = remainder.find("[")
            core_part = remainder[:bracket_idx].strip()
            meta_part = remainder[bracket_idx:].strip()
            meta_dict = self._parse_bracket_metadata(meta_part, ALLOWED_CONN_META_TAGS, line_num)
        else:
            core_part = remainder.strip()

        if "-" not in core_part or len(core_part.split("-")) != 2:
            raise ParsingError(
                line_num,
                f"Malformed connection syntax '{remainder}' (expected: <zone1>-<zone2> [metadata])"
            )

        u_name, v_name = core_part.split("-", 1)
        u_name = u_name.strip()
        v_name = v_name.strip()

        if not u_name or not v_name:
            raise ParsingError(line_num, f"Malformed connection endpoints in '{remainder}'")

        if u_name == v_name:
            raise ParsingError(line_num, f"Self-connection is forbidden: '{u_name}-{v_name}'")

        # Crucial Rule: Connections must link only PREVIOUSLY DEFINED zones!
        if u_name not in graph.nodes:
            raise ParsingError(
                line_num,
                f"Connection start zone '{u_name}' has not been defined yet (zones must precede connections)"
            )

        if v_name not in graph.nodes:
            raise ParsingError(
                line_num,
                f"Connection end zone '{v_name}' has not been defined yet (zones must precede connections)"
            )

        # Validate max_link_capacity
        cap = 1
        if "max_link_capacity" in meta_dict:
            cap_str = meta_dict["max_link_capacity"]
            if not cap_str.isdigit() or int(cap_str) < 1:
                raise ParsingError(line_num, f"max_link_capacity must be a positive integer (> 0), got '{cap_str}'")
            cap = int(cap_str)

        edge = Edge(start=u_name, end=v_name, capacity=cap)
        try:
            graph.add_edge(edge)
        except GraphError as err:
            raise ParsingError(line_num, str(err))

    def parseFile(self, graph: Graph, filename: str) -> None:
        """Backward-compatible camelCase parsing method."""
        self.parse_file(graph, filename)
