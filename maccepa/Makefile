SHELL  =/bin/bash
CC     =gcc
CFLAGS =-Iinclude

# always make
default: m-files/model_maccepa.$(shell mexext) \

build/lib%.o: src/lib%.c include/lib%.h sketchbook/%/defines.h
	$(CC) -o $@ -c $< $(CFLAGS) -Isketchbook/$(subst .c,,$(subst src/lib,,$<)) -fPIC
m-files/model_%.$(shell mexext): build/lib%.o src/mex_lib%.c 	
	mex -output $@ $^ -DMEX_INTERFACE $(CFLAGS) -Isketchbook/$(subst .o,,$(subst build/lib,,$<))

clean:
	rm -rf build/*.o build/*.c build/*.cpp m-files/*.$(shell mexext)

