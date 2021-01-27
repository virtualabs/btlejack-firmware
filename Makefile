YOTTA=yt

all:	ble400 microbit pca10031

ble400: dist .yotta.json
	$(YOTTA) build --config config/ble400.json
	cp build/bbc-microbit-classic-gcc/source/ubit-ble-sniffer-combined.hex dist/btlejack-firmware-ble400.hex
	$(YOTTA) clean

microbit: dist .yotta.json
	$(YOTTA) build
	cp build/bbc-microbit-classic-gcc/source/ubit-ble-sniffer-combined.hex dist/btlejack-firmware-microbit.hex
	$(YOTTA) clean

pca10031: dist .yotta.json
	$(YOTTA) build --config config/pca10031.json
	cp build/bbc-microbit-classic-gcc/source/ubit-ble-sniffer-combined.hex dist/btlejack-firmware-pca10031.hex
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
