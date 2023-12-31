import re
from collections.abc import Iterable
from os import path
from pathlib import Path


class _CmdFileReader:
    def __init__(self, build_root: str, c_file: str) -> None:
        self.c_file = c_file

        dir_name, file_name = path.split(c_file)
        # TODO krri handle .cpp files?
        file_name_base = file_name[:-2]
        self._cmd_file = f'{build_root}/{dir_name}/.{file_name_base}.o.cmd'
        self._values = self._read_cmd_file(self._cmd_file)

        dep_key = f'deps_{dir_name}/{file_name_base}.o'
        self._dependencies = self._clean_deps(self._values[dep_key].split())

    def _read_cmd_file(self, cmd_file: str) -> dict[str, str]:
        result: dict[str, str] = {}

        with open(cmd_file, 'r', encoding='utf8') as f:
            lines = f.readlines()

            key = ''
            value = ''
            is_reading_value = False
            for line in lines:
                remaining = line
                if not is_reading_value:
                    if ':=' in line:
                        parts = line.split(':=')
                        key = parts[0].strip()
                        is_reading_value = True
                        remaining = parts[1]
                    else:
                        continue
                else:
                    remaining = line

                if '\\' in remaining:
                    value += ' ' + remaining.replace('\\', '').strip()
                    is_reading_value = True
                else:
                    value += ' ' + remaining.strip()
                    result[key] = value
                    is_reading_value = False
                    key = ''
                    value = ''

        return result

    def _clean_deps(self, deps: list[str]) -> dict[str, str]:
        result: dict[str, str] = {}

        for dep in deps:
            # Only keep dependencies in our tree
            if dep.startswith('../'):
                file_name = path.basename(dep)

                # Fix relative path by removing ../
                clean_dep = dep[3:]

                result[file_name] = clean_dep

        return result

    def map_to_dependency(self, include_file) -> None | str:
        if include_file in self._dependencies:
            return self._dependencies[include_file]
        return None


class _DependencyNode:
    def __init__(self, source_file: str) -> None:
        self.source_file = source_file
        self.name = path.basename(source_file)
        self.is_processed = False
        self.children: set['_DependencyNode'] = set()
        self.parents: set['_DependencyNode'] = set()
        self.is_h_file = source_file.endswith('.h')
        self.is_c_file = source_file.endswith('.c')

    def add_child(self, child: "_DependencyNode"):
        self.children.add(child)

    def add_parent(self, parent: "_DependencyNode"):
        self.parents.add(parent)

    def mark_processed(self) -> None:
        self.is_processed = True

    def print(self, level=0) -> None:
        for i in range(level):
            print('  ', end='')
        print(self.name, end='')
        if not self.is_processed:
            print(" (not processed)")
        print()


class DependencyGraph:
    """
    A DependencyGraph represents the dependencies in the source tree, all files or a subset. The graph is built from
    .c and .h files, but also .cmd files generated by kbuild, this ensures that the graph is valid for the current
    configuration. It also means that the dependency graph can only be built after a successful make.
    """
    def __init__(self, source_root: str = '.',
                 build_root: str = 'build',
                 exclude_dirs: Iterable[str] = ('src/lib', 'vendor')) -> None:
        """
        Create an empty Dependency graph.

        C-files in excluded directories will never be included. Dependencies to h-files in excluded directories will be
        included to the first level, but not further.

        Args:
            source_root (str, optional): The root of the file tree. Defaults to '.'.
            build_root (str, optional): The root of the build directory generated by kbuild. Defaults to 'build'.
            exclude_dirs (Iterable[str], optional): Directories to exclude. Defaults to ('src/lib', 'vendor').
        """
        self._dep_graph: dict[str, _DependencyNode] = {}
        self._ignored_files: list[str] = []

        self._source_root = source_root
        self._build_root = build_root
        self._exclude_dirs = exclude_dirs

    def add_and_process_c_file(self, c_file: str) -> None:
        """
        Add a c-file with dependencies to the graph

        Args:
            c_file (str): path to the c-file to add
        """
        try:
            cmd_reader = _CmdFileReader(self._build_root, c_file)
        except FileNotFoundError:
            self._ignored_files.append(c_file)
            return

        self._add_to_graph(c_file)
        self._process_nodes(cmd_reader, self._source_root)

    def add_and_process_dir(self, dir_name: str) -> None:
        """
        Add all c-files with dependencies in a directory with sub-directories to the dependency graph.

        Args:
            dir_name (str): path to the directory
        """
        for p in Path(dir_name).rglob('*.c'):
            file_name = str(p)
            if self._include_file(file_name):
                self.add_and_process_c_file(file_name)

    def print_tree(self, file_name: str) -> None:
        """
        Print a dependency tree, starting from a file.

        Note: the same file might be printed multiple times as this is a tree derived from the graph.

        Args:
            file_name (str): The file to start from
        """
        node = self._dep_graph[file_name]
        self._print_node(node)

    def get_file_count(self) -> int:
        """
        Get total number of files in the graph

        Returns:
            int: the number of files
        """
        return len(self._dep_graph)

    def get_c_file_count(self) -> int:
        """
        Get the number of c-files in the graph

        Returns:
            int: the number of files
        """
        c_files = filter(lambda node: node.is_c_file, self._dep_graph.values())
        return len(list(c_files))

    def get_h_file_count(self) -> int:
        """
        Get the number of h-files in the graph

        Returns:
            int: the number of files
        """
        c_files = filter(lambda node: node.is_h_file, self._dep_graph.values())
        return len(list(c_files))

    def get_ignored_files(self) -> list[str]:
        """
        Get a list of all ignored files

        Returns:
            list[str]: List of ignored files
        """
        return self._ignored_files

    def find(self, target: str) -> "DependencyGraph":
        """
        Find the subset defined by the target

        Args:
            target (str): The search string

        Returns:
            DependencyGraph: The subset
        """
        result = DependencyGraph()
        for node in self._find_targets(target):
            result._append(node)
        return result

    def depends_on(self, target: str, levels: int) -> "DependencyGraph":
        """
        Find the subset that target depends on

        Args:
            target (str): the target
            levels (int): the maximum number of levels to follow. Negative means no limit.

        Returns:
            DependencyGraph: _description_
        """
        result = DependencyGraph()
        if levels != 0:
            for node in self._find_targets(target):
                self._append_depends_on(result, node, levels, 1)
        return result

    def used_by(self, target: str, levels: int) -> "DependencyGraph":
        """
        Find the subset that uses target

        Args:
            target (str): the target
            levels (int): the maximum number of levels to follow. Negative means no limit.

        Returns:
            DependencyGraph: _description_
        """
        result = DependencyGraph()
        if levels != 0:
            for node in self._find_targets(target):
                self._append_used_by(result, node, levels, 1)
        return result

    def with_dependency_to(self, other: "DependencyGraph") -> "DependencyGraph":
        """
        Select all nodes that has a dependency to other set

        Args:
            other (DependencyGraph): The other set

        Returns:
            DependencyGraph: The result set
        """
        result = DependencyGraph()

        other_set: set[_DependencyNode] = set(other._dep_graph.values())

        for node in self._dep_graph.values():
            for child in node.children:
                if child in other_set:
                    result._append(node)
                    result._append(child)

        return result

    def union(self, other: "DependencyGraph"):
        """
        Create the union of this and another graph, that is, add the elements of the other graph to this graph.

        Args:
            other (DependencyGraph): Another graph
        """
        for node in other._dep_graph.values():
            self._append(node)

    def export(self) -> tuple[dict[str, dict[str, str | int | bool]], list[list[str]]]:
        nodes: dict[str, dict[str, str | int | bool]] = {}
        edges: list[list[str]] = []
        categories: dict[str, int] = {}

        for file_name, node in self._dep_graph.items():
            category = self._get_category(file_name, categories)
            nodes[file_name] = {
                'name': node.name,
                'file_name': file_name,
                'is-c-file': node.is_c_file,
                'category': category}

            for child in node.children:
                if child.source_file in self._dep_graph:
                    edges.append([node.source_file, child.source_file])

        return nodes, edges

    def _find_targets(self, target: str) -> list[_DependencyNode]:
        result: list[_DependencyNode] = []

        if target in self._dep_graph:
            result.append(self._dep_graph[target])

        if len(result) == 0:
            for node in self._dep_graph.values():
                if node.name == target:
                    result.append(node)

        if len(result) == 0:
            for node in self._dep_graph.values():
                if re.fullmatch(target, node.source_file):
                    result.append(node)

        return result

    def _append_depends_on(self, result: "DependencyGraph", node: _DependencyNode, max_level: int, curr_level: int):
        for child in node.children:
            if result._append(child):
                if curr_level < max_level or max_level < 0:
                    self._append_depends_on(result, child, max_level, curr_level + 1)

    def _append_used_by(self, result: "DependencyGraph", node: _DependencyNode, max_level: int, curr_level: int):
        for parent in node.parents:
            if result._append(parent):
                if curr_level < max_level or max_level < 0:
                    self._append_used_by(result, parent, max_level, curr_level + 1)

    def _get_category(self, source_file: str, categories: dict[str, int]) -> int:
        result = 0
        parts = source_file.split('/')
        if len(parts) >= 2:
            if parts[0] == 'src':
                fragment = parts[1]

                if fragment not in categories:
                    categories[fragment] = len(categories) + 1
                result = categories[fragment]
        return result

    def _print_node(self, node: _DependencyNode, level=0) -> None:
        node.print(level)
        for child in node.children:
            self._print_node(child, level + 1)

    def _add_to_graph(self, source_file: str) -> _DependencyNode:
        if source_file not in self._dep_graph:
            self._dep_graph[source_file] = _DependencyNode(source_file)
        return self._dep_graph[source_file]

    def _append(self, node: _DependencyNode) -> bool:
        if node.source_file not in self._dep_graph:
            self._dep_graph[node.source_file] = node
            return True
        return False

    def _process_nodes(self, cmd_reader: _CmdFileReader, source_root: str) -> None:
        while True:
            to_process = list(filter(lambda node: not node.is_processed, self._dep_graph.values()))
            for node in to_process:
                self._process_node(cmd_reader, node, source_root)
            if len(to_process) == 0:
                break

    def _process_node(self, cmd_reader: _CmdFileReader, node: _DependencyNode, source_root: str) -> None:
        if self._include_file(node.source_file):
            include_files = self._parse_includes(f'{source_root}/{node.source_file}')
            for include_file in include_files:
                source_file = cmd_reader.map_to_dependency(include_file)
                if source_file:
                    child = self._add_to_graph(source_file)
                    node.add_child(child)
                    child.add_parent(node)

        node.mark_processed()

    def _parse_includes(self, source_file: str) -> list[str]:
        with open(source_file, 'r', encoding="utf8") as f:
            contents = f.read()
            pattern = '#include +(?:"|<)(.*)(?:"|>)'
            return re.findall(pattern, contents)

    def _include_file(self, source_file: str):
        for exclude in self._exclude_dirs:
            if source_file.startswith(exclude):
                return False
        return True
