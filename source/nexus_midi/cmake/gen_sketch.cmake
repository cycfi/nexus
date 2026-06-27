# Turn an Energia .ino into a compilable .cpp by prepending <Energia.h>.
# The IDE normally does this (plus auto-prototypes); this sketch defines every
# function before use, so the include is all that's needed.
#
# Invoked as: cmake -DINO=<in.ino> -DOUT=<out.cpp> -P gen_sketch.cmake
file(READ "${INO}" _sketch)
file(WRITE "${OUT}" "#include <Energia.h>\n${_sketch}")
