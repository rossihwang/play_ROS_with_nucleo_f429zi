all: imu.proto twist.proto encoder.proto Vector3.proto
	protoc --plugin=generator/protoc-gen-nanopb --nanopb_out=. $^