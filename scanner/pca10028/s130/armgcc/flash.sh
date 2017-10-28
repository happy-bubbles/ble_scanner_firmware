openocd -d2 -f x.cfg  -c "init; halt; nrf51 mass_erase; reset; halt; flash write_image erase combined.hex 0; verify_image combined.hex 0; reset; exit;"
