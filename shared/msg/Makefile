all: normal max_size_feature 
	
normal: imu.proto twist.proto encoder.proto Vector3.proto pwm_ctrl.proto
	protoc --plugin=generator/protoc-gen-nanopb --nanopb_out=. $^

max_size_feature: logging.proto
	protoc --plugin=generator/protoc-gen-nanopb -Igenerator/proto -I. --nanopb_out=. $^
