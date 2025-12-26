def ros2_launcher(name, target):
    native.genrule(
        name = name,
        visibility = ["//visibility:public"],
        outs = [name + ".sh"],
        cmd = """
cat << 'EOF' > $@
#!/bin/bash
# Use $$ to escape the dollar sign for Bazel
DIR=$$(cd "$$(dirname "$${BASH_SOURCE[0]}")" && pwd)

# Find the specific runfiles folder for the target
RF_DIR="$$DIR/""" + target + """.runfiles"

# Find the middleware library inside that tree
MW_LIB=$$(find "$$RF_DIR" -name "librmw_cyclonedds.so" | head -n 1)

export LD_PRELOAD="$$MW_LIB"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec "$$DIR/""" + target + """" "$$@"
EOF
""",
        executable = True,
    ) 