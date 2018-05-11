CC=gcc
CFLAGS=-I.
DEPS= 
LIBFLAGS=-lpthread

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

dbgsocket: dbgsocket.o 
	gcc -o dbgsocket dbgsocket.o ${LIBFLAGS} -I.
