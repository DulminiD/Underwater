from crc import CrcCalculator, Crc8

bits = [1,1,1,0,0,1,0,0]
bits = bits * 6
data = bytes(bits)
expected_checksum = 0xBC
crc_calculator = CrcCalculator(Crc8.CCITT)
checksum = crc_calculator.calculate_checksum(data)
print(checksum)