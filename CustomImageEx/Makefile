# Handle environment variables

OPENCV    = `pkg-config --libs opencv`
EXECUTABLES = CustomImageEx
INC       =  -I/usr/include/flycapture
LIBS      =  -lflycapture 
CXX       = g++
OBJ      = 

# Master rule
all: ${EXECUTABLES}

# Output binary
$(EXECUTABLES): % : %.c $(OBJ)
	$(CXX) -o $@ $< $(OBJ) $(LIBS) $(INC) $(OPENCV)

# Intermediate object files
$(OBJ): %.o : %.cpp
	$(CXX) $(INC) $(OPENCV) -Wall -c $< 

# Cleanup intermediate objects
.PHONY: clean_obj
clean_obj:
	rm -f *.o 
	@echo "obj cleaned up!"

# Cleanup everything
.PHONY: clean
clean: clean_obj
	rm -f ${EXECUTABLES} *.o 
	@echo "all cleaned up!"

