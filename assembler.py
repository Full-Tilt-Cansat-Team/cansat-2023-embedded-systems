# This program assembles our documentation into code, then runs the compiler

# First, we need to get a list of all of the .md files in the documentation folder
# As well as any .md files in the subfolders
import os
import sys

files = []

for root, dirs, filenames in os.walk("documentation"):
    for f in filenames:
        if f.endswith(".md"):
            files.append(os.path.join(root, f))

# This object will handle code blocks, such as storing the code, as well as a tag for it's position
class CodeBlock:
    def __init__(self, code, tag):
        self.code = code
        self.tags = tag

# Walk through all of our files, grab their text and look for code blocks
# Code blocks start and end with ```, and their first line includes the tag for parsing, split by a space
codeBlocks = []

for f in files:
    with open(f, "r") as file:
        text = file.readlines()
        currentBlock = []
        reading = False

        for line in text:
            if line.startswith("```"):
                if reading:
                    tags = currentBlock[0].split(" ")
                    for i in range(len(tags)):
                        if tags[i][-1] == "\n":
                            tags[i] = tags[i][:-1]
                        if tags[i].startswith("//"):
                            tags[i] = tags[i][2:]

                    codeBlocks.append(CodeBlock(currentBlock[1:], tags))
                    currentBlock = []
                    reading = False
                else:
                    reading = True
            elif reading:
                currentBlock.append(line[:-1])

for i in codeBlocks:
    print(i.code)
    print(i.tags)

# The first tag we block out is the import tag
# These blocks handle package importing
# No special processing is used here
import_lines = []

for block in codeBlocks:
    if block.tags[0] == "import":
        for i in range(len(block.code)):
            import_lines.append(block.code[i])

        import_lines.append("")

# The next tag we block out is the init tag
# This is the same as the import tag, but is proccesed after the import tag
# This is used for initializing variables, usually global ones
init_lines = []

for block in codeBlocks:
    if block.tags[0] == "init":
        for i in range(len(block.code)):
            init_lines.append(block.code[i])

        init_lines.append("")
        
# Now we'll pre-process the func-def tags. These are used to define functions, but we need
# to remove the function closing tag
for i in range(len(codeBlocks)):
    if codeBlocks[i].tags[0] == "func-def":
        codeBlocks[i].code = codeBlocks[i].code[:-1]

# Now, append any func tag blocks to the end of the function they're defined in
# The func tag's function name is the second tag,same as the func-def tag
for i in range(len(codeBlocks)):
    if codeBlocks[i].tags[0] == "func":
        for j in range(len(codeBlocks)):
            if codeBlocks[j].tags[0] == "func-def":
                if codeBlocks[j].tags[1] == codeBlocks[i].tags[1]:
                    for k in range(len(codeBlocks[i].code)):
                        codeBlocks[j].code.append(codeBlocks[i].code[k])

# Now, finally we can add a singular function list
functions = []

for block in codeBlocks:
    if block.tags[0] == "func-def":
        for line in block.code:
            functions.append(line)

        functions.append("}")

# Now, write the code to a compile/src/main.rs file
with open("compile/src/main.rs", "w") as file:
    for i in import_lines:
        file.write(i + "\n")

    file.write("\n")

    for i in init_lines:
        file.write(i + "\n")
    
    file.write("\n")

    for i in functions:
        file.write(i + "\n")
    
    file.write("\n")

# Now, we can compile the code
# We'll use the rust compiler, and the rust compiler will handle the rest
os.system("cd compile && cargo run")