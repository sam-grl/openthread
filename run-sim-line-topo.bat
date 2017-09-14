@echo off
for /l %%j in (1, 1, 11) do (
    @echo Starting OpenThread node %%j
	@run.bat %%j
)

