"""Old GroIMP CSV -> BRANCHES builder.

Not an exporter input any more (the exporter reads plant skeletons). It is
kept only because the core tests build their test plants with it.
"""

from .parser import parse_csv_to_branches

__all__ = ["parse_csv_to_branches"]
