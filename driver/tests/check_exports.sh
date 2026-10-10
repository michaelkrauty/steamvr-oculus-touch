#!/bin/sh
# The driver exports only HmdDriverFactory and needs no C++ runtime from the host: vrserver
# loads it beside SteamVR's own runtime libraries (Review Focus 3).
set -e
so="$1"
exports=$(nm -D --defined-only "$so" | awk '{print $NF}' | grep -v -E '^(_init|_fini|__bss_start|_edata|_end)$' || true)
if [ "$exports" != "HmdDriverFactory" ]; then
	echo "unexpected exports:"; echo "$exports"; exit 1
fi
if readelf -d "$so" | grep -E 'NEEDED.*(libstdc\+\+|libgcc_s)'; then
	echo "links the C++ runtime dynamically"; exit 1
fi
if nm -D "$so" | grep -E '@(GLIBCXX|CXXABI)_'; then
	echo "needs C++ runtime symbol versions"; exit 1
fi
echo "exports ok"
