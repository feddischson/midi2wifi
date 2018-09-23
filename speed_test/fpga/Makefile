PRJ=speed_test

QPF := $(PRJ).qpf

SOF_FILE  		:= $(PRJ).sof
SOPC_FILE 		:= $(PRJ).sopcinfo
RBF_FILE 		:= $(PRJ).rbf
CDF_FILE 		:= $(PRJ).cdf   
JIC_FILE 		:= $(PRJ).jic
CDF_JIC_FILE 	:= $(PRJ)_jic.cdf
COF_FILE 		:= $(PRJ).cof

SRC = ${PRJ}.v

.PHONY: all
all: rbf quartus_generate  

.PHONY: rbf
rbf: $(RBF_FILE)

.PHONY: jic
rbf: $(JIC_FILE)

.PHONY: quartus_edit
quartus_edit:
	quartus $(QPF) &

.PHONY: quartus_generate
quartus_generate:  $(SOF_FILE)

.PHONY: programm
programm: $(SOF_FILE) $(CDF_FILE)
	quartus_pgm $(CDF_FILE)

.PHONY: flash
flash: $(JIC_FILE) $(CDF_JIC_FILE)
	quartus_pgm $(CDF_JIC_FILE)

$(JIC_FILE): $(SOF_FILE)
	quartus_cpf -c $(COF_FILE)

$(RBF_FILE): $(SOF_FILE)
	quartus_cpf -c $(SOF_FILE) $(RBF_FILE)

$(SOF_FILE): $(SRC)
	quartus_sh --flow compile  $(QPF)


clean:
	rm -rf 	db \
			incremental_db \
	        c5_pin_model_dump.txt \
	        *_assignment_defaults.qdf \
			.qsys_edit \
			$(SOF_FILE) \
			$(RBF_FILE)\
			*.rpt \
			*.done \
			*.smsg \
			*.summary \
			*.htm \
			*.sopcinfo \
			*.sdl \
			*.pin \
			*.qws \
			*.sld \
			*.map \
			*.jdi \
			*.jic

