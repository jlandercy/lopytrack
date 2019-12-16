function Decode(port, bytes) {
  /*
  Decode function for LoRaServer (ChirpStack)
  */
  function BytesToInt16(b) {
    /*
    Assemble Int16 from 2 bytes

    References:

      - https://en.wikipedia.org/wiki/Integer_(computer_science)
    
    Endianness (Byte Order):

      Python and JavaScript have the same endianness than the processor it executes on.
      Developper should write codes compliant with both scenarii, as it would break if it is executed elsewhere.
      To control the byte order in Python, make use of struct.pack method with right type and angle bracket (< or >).

        >>> import struct
        >>> struct.pack('<h', 1)
        b'\x01\x00'

      Then transpose the correct byte order in JavaScript by shifting properly (<< or >>).

        var bits = b[0]<<8 | b[1];

      References:

        - https://en.wikipedia.org/wiki/Endianness
        - https://developer.mozilla.org/en-US/docs/Glossary/Endianness
        - https://docs.python.org/3.7/library/struct.html#byte-order-size-and-alignment
        - https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Operators/Bitwise_Operators
    */
    // Assemble bytes:
    var bits = b[0]<<8 | b[1];
    return bits;
  };
  function BytesToFloat32(bytes) {
    /*
    Assemble Float32 from 4 bytes

    References:

      - https://en.wikipedia.org/wiki/IEEE_754
      - https://en.wikipedia.org/wiki/NaN
    */
    // Assemble bytes:
    var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
    // Exponent:
    var e = bits>>>23 & 0xff;
    // Detect NaN: s111 1111 1xxx xxxx xxxx xxxx xxxx xxxx
    if(e == 255){
      // Float32 NaN reads as 5.104235503814077e+38 in Float64, therefore it produces large outliers
      // Additionally, NaN are not allowed in JSON representation, use null instead
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
