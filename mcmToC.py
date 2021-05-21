import sys

print("Hello Python")

if (len(sys.argv) != 3):
    print("Usage: mcmToC <inputFile> <outputFile>")
    quit()

print(sys.argv[1])

inputFile = open(sys.argv[1])
outputFile = open(sys.argv[2], "w")

# The first line should be "MAX7456"
line = inputFile.readline()
# if ("MAX7456" not in line):
if (line.strip() != "MAX7456"):
    print("File doesn't start with MAX7456 - wrong format?")
    print(line)
    quit()

# Input format is 4 pixels per line in binary (2 bits per pixel so 1 byte per line).
# 3 lines make a character row and
# 18 rows make a full character (12 x 18 pixels), but there are also extra padding
# bytes at the end of each character to take it up to 64 bytes instead of 54

# output format is a C array of characters where each character is 18 uint_32

outputFile.write("uint32_t font[][18] = {\n")


characters = 0
lineCount = 0

line = "x"
while line != "":

    outputFile.write("{") # start of character

    row = 0
    while(row < 18):
        # each row requires three lines each containing 4 pixels
        print("read row")

        highBits = inputFile.readline().strip()
        middleBits = inputFile.readline().strip()
        lowBits = inputFile.readline().strip()

        lineCount = lineCount + 3

        if highBits == "":
            line = ""
            break

        print(lineCount, highBits, middleBits, lowBits)

        # convert to 32 bit value
        rowA = int(highBits, 2) << 16
        rowB = int(middleBits, 2) << 8
        rowC = int(lowBits, 2)
        print(hex(rowA), hex(rowB), hex(rowC))
        print(hex(rowA+rowB+rowC))

        outputFile.write(hex(rowA+rowB+rowC)) 
        if row < 17:
            outputFile.write(", ")

        row += 1

    outputFile.write("},\n") # end of character

    characters = characters + 1

    # need to discard the 10 lines of padding at the end of each character
    discardCount = 0
    while(discardCount < 10):
        inputFile.readline()
        discardCount = discardCount + 1


outputFile.write("};\n") # end of array

outputFile.write("#define FONT_NCHAR "+str(characters)+"\n")

print("done")
