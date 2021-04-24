/* ChirpStack Application payload decoder */
function Decode(port, bytes) {
  return {
    "error": (bytes[0] & 0x01),
    "fixed": (bytes[0] & 0x02) >> 1,
    "time": bytes[1] | (bytes[2] << 8) | (bytes[3] << 16),
    "longitude": (bytes[5] | (bytes[6] << 8) | (bytes[7] << 16) | (bytes[8] << 24))/1000000.,
    "latitude": (bytes[9] | (bytes[10] << 8) | (bytes[11] << 16) | (bytes[12] << 24))/1000000.,
    "height": bytes[13] | (bytes[14] << 8),
    "precision": (bytes[15] | (bytes[16] << 8))/10.
  };
};

/* TTN Application payload decoder */
function Decoder(bytes, port) {
  return Decode(port, bytes);
};
