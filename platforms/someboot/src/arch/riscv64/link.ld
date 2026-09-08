PECOFF_FILE_ALIGN = 0x200;

${ld_defaults}
VM_LOAD_ADDRESS = ${kernel_load_vaddr};

OUTPUT_ARCH(riscv)
ENTRY(_head)

SECTIONS
{
    . = VM_LOAD_ADDRESS;
    .head.text : {
        KEEP(*(.head.text.000.header))
        KEEP(*(.head.text))
        KEEP(*(SORT_BY_NAME(.head.text.*)))
        KEEP(*(.text._head))
    }
${ld_text}
${ld_rodata}
${ld_data}
${ld_rela_dyn}
    .dynsym : ALIGN(8) {
        *(.dynsym)
    }
    .gnu.hash : ALIGN(8) {
        *(.gnu.hash)
    }
    .hash : ALIGN(8) {
        *(.hash)
    }
    .dynstr : ALIGN(1) {
        *(.dynstr)
    }
    .eh_frame_hdr : ALIGN(4) {
        *(.eh_frame_hdr)
    }
${ld_bss}

    _kernel_asize = ABSOLUTE(_end - _head);
    _kernel_fsize = ABSOLUTE(_edata - _head);
    _kernel_vsize = ABSOLUTE(_end - _etext);
    _kernel_rsize = ABSOLUTE(_edata - _etext);
    _kernel_code_size = ABSOLUTE(_etext - _efi_header_end);
    _kernel_bss_size = ABSOLUTE(_kernel_vsize - _kernel_rsize);
    _kernel_text_offset = ABSOLUTE(_efi_header_end - _head);
    _kernel_image_size = ABSOLUTE(_end - _head);

    /DISCARD/ : {
        *(.eh_frame)
        *(.note .note.*)
        *(.comment)
    }
}
