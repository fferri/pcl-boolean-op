PCL_MODULES=common filters io kdtree sample_consensus
PCL_VERSION=1.7
PCL_CFLAGS = $(shell sh -c 'for i in $(PCL_MODULES); do pkg-config pcl_$${i}-$(PCL_VERSION) --cflags; done')
PCL_LDLIBS = $(shell sh -c 'for i in $(PCL_MODULES); do pkg-config pcl_$${i}-$(PCL_VERSION) --libs; done')

CFLAGS := -std=c++03 -O0 -ggdb -I/opt/local/include $(PCL_CFLAGS) $(OPT_CFLAGS)
LDLIBS := -lm -lstdc++ -L/opt/local/lib $(PCL_LDLIBS)

ifeq ($(OS),Windows_NT)
    CFLAGS += -DWIN32
    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
        CFLAGS += -DAMD64
    endif
    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
        CFLAGS += -DIA32
    endif
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        CFLAGS += -DLINUX
		UNAME_P := $(shell uname -p)
		ifeq ($(UNAME_P),x86_64)
			CFLAGS += -DAMD64
		endif
		ifneq ($(filter %86,$(UNAME_P)),)
			CFLAGS += -DIA32
		endif
		ifneq ($(filter arm%,$(UNAME_P)),)
			CFLAGS += -DARM
		endif
        LDLIBS += -lboost_system -lboost_thread -lboost_filesystem -lboost_program_options
    endif
    ifeq ($(UNAME_S),Darwin)
        CFLAGS += -DOSX
		UNAME_M := $(shell uname -m)
		ifeq ($(UNAME_M),x86_64)
			CFLAGS += -DAMD64
		endif
		ifneq ($(filter %86,$(UNAME_M)),)
			CFLAGS += -DIA32
		endif
        LDLIBS += -lboost_system-mt -lboost_thread-mt -lboost_filesystem-mt -lboost_program_options-mt
    endif
endif

CXXFLAGS = $(CFLAGS)

.PHONY: clean all install

OBJS = pcl_union.o pcl_intersection.o pcl_difference.o pcl_symmetric_difference.o pcl_common.o main.o pcl_union_fast.o pcl_make_scanlog.o pcl_remove_dynamic_obstacles.o pcl_jaccard_similarity.o

TARGETS = pcl_union pcl_intersection pcl_difference pcl_symmetric_difference pcl_union_fast pcl_make_scanlog pcl_remove_dynamic_obstacles pcl_jaccard_similarity

PREFIX = /usr/local

all: $(TARGETS)

install: $(TARGETS)
	for i in $(TARGETS); do cp -v $$i $(PREFIX)/bin/; done

pcl_union: pcl_union.o pcl_common.o main.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_intersection: pcl_intersection.o pcl_common.o main.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_difference: pcl_difference.o pcl_common.o main.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_symmetric_difference: pcl_symmetric_difference.o pcl_common.o main.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_union_fast: pcl_union_fast.o pcl_common.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_make_scanlog: pcl_make_scanlog.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_remove_dynamic_obstacles: pcl_remove_dynamic_obstacles.o
	$(CC) $^ $(LDLIBS) -o $@

pcl_jaccard_similarity: pcl_jaccard_similarity.o pcl_common.o
	$(CC) $^ $(LDLIBS) -o $@

-include $(OBJS:.o=.d)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $*.cpp -o $*.o
	$(CXX) -MM $(CXXFLAGS) $*.cpp > $*.d
	@cp -f $*.d $*.d.tmp
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

clean:
	@rm -vf $(OBJS) $(OBJS:.o=.d) pcl_union pcl_intersection pcl_difference
	@sh -c 'if ls *.o > /dev/null 2>&1; then echo "error: some *.o files have not been cleaned: $$(ls *.o)"; false; fi'

