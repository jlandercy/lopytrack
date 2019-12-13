function Decode(port, bytes) {
    function bytesToFloat(bytes) {
      // JavaScript bitwise operators yield a 32 bits integer, not a float.
      // Assume LSB (least significant byte first).
      var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
      var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
      var e = bits>>>23 & 0xff;
      var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
      var f = sign * m * Math.pow(2, e - 150);
      return f;
    };
    // Compose sensor object:
    return {
      "lat": bytesToFloat(bytes.slice(0,4)),
      "lon": bytesToFloat(bytes.slice(4,8)),
      "ax": bytesToFloat(bytes.slice(8,12)),
      "ay": bytesToFloat(bytes.slice(12,16)),
      "az": bytesToFloat(bytes.slice(16,20)),
      "roll": bytesToFloat(bytes.slice(20,24)),
      "pitch": bytesToFloat(bytes.slice(24,28)),
      "count": bytesToFloat(bytes.slice(28,32)),
      "missed": bytesToFloat(bytes.slice(32,36)),
    };
}
  