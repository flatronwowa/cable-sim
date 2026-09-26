#!/usr/bin/env python

env = SConscript("godot-cpp/SConstruct")

env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp") + Glob("src/nodes/*.cpp") + Glob("src/solver/*.cpp")

# Godot generates docs for GDExtension classes on load; without linked doc data
# the editor crashes in EditorHelp::_gen_extensions_docs().
if env["target"] in ["editor", "template_debug"]:
    sources.append(env.GodotCPPDocData("src/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml")))

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "project/bin/libcable.{}.{}.framework/libcable.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "project/bin/libcable{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

env.NoCache(library)
Default(library)
