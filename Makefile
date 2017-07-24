CC	=	g++ -Wall
OPENCV =  `pkg-config --libs opencv` 
BINDIR = bin
DBGDIR = debug
OBJDIR = .obj
SRCDIR = src
SOURCES_C := $(wildcard $(SRCDIR)/*.c)
SOURCES_CPP  := $(wildcard $(SRCDIR)/*.cpp)
OBJECTS  := $(SOURCES_CPP:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
EXECUTABLES := $(SOURCES_C:$(SRCDIR)/%.c=$(BINDIR)/%)
DBGEXECUTABLES := $(SOURCES_C:$(SRCDIR)/%.c=$(DBGDIR)/%)

LIBS = -lflycapture ${OPENCV}
INCS = -I /usr/include/flycapture


DBGCFLAGS = -g -O0 -DDEBUG


CXXFLAGS = -g3 -gdwarf2
CCFLAGS = -g3 -gdwarf2

##Default
all: prep release

release: $(EXECUTABLES)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(LIBS) $(INCS)

$(EXECUTABLES): $(BINDIR)/% : $(SRCDIR)/%.c $(OBJECTS)
	$(CC) -o $@ $< $(OBJECTS) $(LIBS) $(INCS)

#
# Debug rules
#
debug: $(DBGEXECUTABLES)
debug: CXXFLAGS += -DDEBUG -g
debug: CCFLAGS += -DDEBUG -g

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(LIBS) $(INCS)

$(DBGEXECUTABLES): $(DBGDIR)/% : $(SRCDIR)/%.c $(OBJECTS)
	$(CC) -o $@ $< $(OBJECTS) $(LIBS) $(INCS)


.PHONY: all clear

#
# Other rules
#
prep:
	@mkdir -p $(DBGDIR) $(BINDIR) $(OBJDIR)

clean:
	rm $(EXECUTABLES)
	rm -rf $(DBGDIR)
	rm $(OBJECTS)


