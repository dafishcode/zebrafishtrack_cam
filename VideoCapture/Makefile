################################################################################
# FlyCapture2Test Makefile.
################################################################################
PROJECT_ROOT=.
# Handle environment variables

OPENCV    = `pkg-config --libs opencv`
BASE_NAME = SaveImageSeq
NAME      = ${BASE_NAME}${D}
INC       =  -I/usr/include/flycapture
LIBS      =  -lflycapture${D} ${FC2_DEPS}
OUTDIR    = ${PROJECT_ROOT}

OBJ      = ${BASE_NAME}.o

# Master rule
.PHONY: all
all: ${NAME}

# Output binary
${NAME}: ${OBJ}
	${CXX} ${OBJ} -o ${NAME} ${LIBS} ${OPENCV}

# Intermediate object files
${OBJ}: %.o : %.cpp
	${CXX} ${CXXFLAGS} ${LINUX_DEFINES} ${INC} ${OPENCV} -Wall -c $< -o $@

# Cleanup intermediate objects
.PHONY: clean_obj
clean_obj:
	rm -f *.o 
	@echo "obj cleaned up!"

# Cleanup everything
.PHONY: clean
clean: clean_obj
	rm -f ${NAME} *.o 
	@echo "all cleaned up!"

