

GCC_CMD=riscv64-elf-gcc
TEST_OBJS = $(addprefix build/,$(addsuffix .o,$(notdir $(basename $(wildcard riscv_tests/*.S)))))

$(info $$TEST_OBJS is [${TEST_OBJS}])

build/firmware_tests.elf: $(TEST_OBJS) blockram.ld
	$(GCC_CMD) -Os -mabi=ilp32 -march=rv32i -ffreestanding -nostdlib -o $@ \
		-Wl,--build-id=none,-Bstatic,-T,blockram.ld,-Map,build/firmware.map,--strip-debug \
		$(TEST_OBJS) -lgcc
	chmod -x $@

build/start.o: start.S
	$(GCC_CMD) -c -mabi=ilp32 -march=rv32i -o $@ $<

build/%.o: riscv_tests/%.S riscv_tests/riscv_test.h riscv_tests/test_macros.h
	$(GCC_CMD) -c -mabi=ilp32 -march=rv32i -o $@ -DTEST_FUNC_NAME=$(notdir $(basename $<)) -DTEST_FUNC_TXT='"$(notdir $(basename $<))"' -DTEST_FUNC_RET=$(notdir $(basename $<))_ret $<

