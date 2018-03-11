for /l %%x in (0, 1, 100) do (
srec_cat -generate 0x0DFC 0x0DFD -constan-l-e %%x 1 Debug\NespressoBot.hex -intel -exclude 0x0DFC 0x0DFD -o bins\weddingbot%%x.hex -intel -line-length=44 -address-length=2
)