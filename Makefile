YOTTA=yt

all:	ble400 bornhack2018 microbit

ble400: dist .yotta.json
	$(YOTTA) build --config config/ble400.json
	cp build/bbc-microbit-classic-gcc/source/ubit-ble-sniffer-combined.hex dist/btlejack-firmware-ble400.hex
	$(YOTTA) clean

bornhack2018: dist .yotta.json
	$(YOTTA) build --config config/bornhack2018.json
	cp build/bbc-microbit-classic-gcc/source/ubit-ble-sniffer-combined.hex dist/btlejack-firmware-bornhack2018.hex
	$(YOTTA) clean

microbit: dist .yotta.json
	$(YOTTA) build
	cp build/bbc-microbit-classic-gcc/source/ubit-ble-sniffer-combined.hex dist/btlejack-firmware-microbit.hex
	$(YOTTA) clean

dist:
	@mkdir -p dist
	@rm dist/* -rf

.yotta.json:
	$(YOTTA) target bbc-microbit-classic-gcc

clean:
	$(YOTTA) clean
	rm dist/ -rf
	rm .yotta.json
