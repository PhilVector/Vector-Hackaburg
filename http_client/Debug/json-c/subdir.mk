################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../json-c/arraylist.c \
../json-c/debug.c \
../json-c/json_c_version.c \
../json-c/json_object.c \
../json-c/json_object_iterator.c \
../json-c/json_pointer.c \
../json-c/json_tokener.c \
../json-c/json_util.c \
../json-c/linkhash.c \
../json-c/printbuf.c \
../json-c/random_seed.c \
../json-c/strerror_override.c 

OBJS += \
./json-c/arraylist.o \
./json-c/debug.o \
./json-c/json_c_version.o \
./json-c/json_object.o \
./json-c/json_object_iterator.o \
./json-c/json_pointer.o \
./json-c/json_tokener.o \
./json-c/json_util.o \
./json-c/linkhash.o \
./json-c/printbuf.o \
./json-c/random_seed.o \
./json-c/strerror_override.o 

C_DEPS += \
./json-c/arraylist.d \
./json-c/debug.d \
./json-c/json_c_version.d \
./json-c/json_object.d \
./json-c/json_object_iterator.d \
./json-c/json_pointer.d \
./json-c/json_tokener.d \
./json-c/json_util.d \
./json-c/linkhash.d \
./json-c/printbuf.d \
./json-c/random_seed.d \
./json-c/strerror_override.d 


# Each subdirectory must supply rules for building sources it contributes
json-c/%.o: ../json-c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


