import operator
import os
import tempfile

import kconfiglib
from jinja2 import Environment, FileSystemLoader


def choice_id(choice):
    return f"choice_{choice.kconfig.unique_choices.index(choice)}"


def choice_desc(choice):
    desc = "choice"

    if choice.name:
        desc += " " + choice.name

    for node in choice.nodes:
        if node.prompt:
            desc += ": " + node.prompt[0]
            break

    return desc


def rst_link(sc):
    if isinstance(sc, kconfiglib.Symbol):
        if sc.nodes:
            return fr"\ :option:`{sc.name} <CONFIG_{sc.name}>`"

    elif isinstance(sc, kconfiglib.Choice):
        return fr"\ :ref:`<{choice_desc(sc)}> <{choice_id(sc)}>`"

    return kconfiglib.standard_sc_expr_str(sc)


def expr_str(expr):
    return kconfiglib.expr_str(expr, rst_link)


def top_to_node(node):
    path = []
    while node.parent is not node.kconfig.top_node:
        node = node.parent
        path = [node] + path
    return path


def escape_help(sc):
    for node in sc.nodes:
        if node.help:
            node.help = (
                node.help.replace("`", r"\`")
                .replace("*", "\*")
                .replace("<", "\<")
                .replace("|", "\|")
            )


def main():
    topdir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
    os.environ["TOPDIR"] = topdir
    os.environ["APPSDIR"] = os.path.realpath(os.path.join(topdir, "..", "apps"))

    open(os.path.join(topdir, "arch", "dummy", "Kconfig"), "w").write("")
    open(os.path.join(topdir, "boards", "dummy", "Kconfig"), "w").write("")
    os.makedirs(os.path.join(topdir, "drivers", "platform"), exist_ok=True)
    open(os.path.join(topdir, "drivers", "platform", "Kconfig"), "w").write("")
    external = tempfile.mkdtemp()
    open(os.path.join(external, "Kconfig"), "w").write("")
    os.environ["EXTERNALDIR"] = os.path.realpath(external)

    kconfig_rst_dir = os.path.join(topdir, "Documentation", "kconfig")

    os.environ["srctree"] = topdir
    kconf = kconfiglib.Kconfig("Kconfig", warn=False)

    for sym in kconf.unique_defined_syms:
        escape_help(sym)

    env = Environment(
        loader=FileSystemLoader(os.path.dirname(__file__)),
    )

    env.globals["kconfiglib"] = kconfiglib
    env.globals["expr_str"] = expr_str
    env.globals["rst_link"] = rst_link
    env.globals["top_to_node"] = top_to_node
    env.globals["choice_desc"] = choice_desc
    env.globals["choice_id"] = choice_id

    sym_template = env.get_template("sym.jinja")
    choice_template = env.get_template("choice.jinja")
    index_template = env.get_template("index.jinja")

    for sym in kconf.unique_defined_syms:
        escape_help(sym)
        with open(
            os.path.join(kconfig_rst_dir, f"CONFIG_{sym.name}.rst"), "w"
        ) as rst_f:
            rst_f.write(sym_template.render(sym=sym))

    for choice in kconf.unique_choices:
        escape_help(choice)
        with open(
            os.path.join(kconfig_rst_dir, f"{choice_id(choice)}.rst"), "w"
        ) as rst_f:
            rst_f.write(choice_template.render(choice=choice))

    with open(os.path.join(kconfig_rst_dir, "index.rst"), "w") as rst_f:
        rst_f.write(
            index_template.render(
                syms=sorted(kconf.unique_defined_syms, key=operator.attrgetter("name")),
            )
        )


if __name__ == "__main__":
    main()
