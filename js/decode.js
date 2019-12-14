function Decode(port, bytes) {
  /*
  Decode function for LoRaServer (ChirpStack)
  */
  function BytesToInt16(b) {
    /*
    Assemble Int16 from 2 bytes (big endian)
    https://en.wikipedia.org/wiki/Integer_(computer_science)
    https://en.wikipedia.org/wiki/Endianness
    NB: Python has the same endianess than the processor it executes on!
    */
    // Assemble bytes:
    bits = b[0]<<8 | b[1]
    return bits;
  };
  function BytesToFloat32(bytes) {
    /*
    Assemble Float32 from 4 bytes (big endian)
    https://en.wikipedia.org/wiki/IEEE_754
    https://en.wikipedia.org/wiki/NaN
    */
    // Assemble bytes:
    var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
    // Exponent:
    var e = bits>>>23 & 0xff;
    // Detect NaN: s111 1111 1xxx xxxx xxxx xxxx xxxx xxxx
    if(e == 255){
      // NaN are not allowed in JSON representation, use null instead
      return null; 
    } else {
      // Sign:
      var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
      // Mantisa:
      var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
      // Compose float as sign * mantisa * 2^(exponent - 150)
      var f = sign * m * Math.pow(2, e - 150);
      return f;
    };
  };
  // Compose sensor object:
  return {
    "time": BytesToFloat32(bytes.slice(0,4)),
    "lat": BytesToFloat32(bytes.slice(4,8)),
    "lon": BytesToFloat32(bytes.slice(8,12)),
    "height": BytesToFloat32(bytes.slice(12,16)),
    "hdop": BytesToFloat32(bytes.slice(16,20)),
    "ax": BytesToFloat32(bytes.slice(20,24)),
    "ay": BytesToFloat32(bytes.slice(24,28)),
    "az": BytesToFloat32(bytes.slice(28,32)),
    "roll": BytesToFloat32(bytes.slice(32,36)),
    "pitch": BytesToFloat32(bytes.slice(36,40))
  };
};

function Decoder(bytes, port) {
  /*
  Decode function for The Things Network (TTN)
  */
  // Adapt function signature Decoder(bytes, port) to Decode(port, bytes):
  return Decode(port, bytes);
};




