use anyhow::Result;

// Helper function to decode a 4-character SCIP2.0 encoded value
pub fn decode_scip_2_0_4char(encoded_val: &str) -> Result<u32, anyhow::Error> {
    if encoded_val.len() != 4 {
        return Err(anyhow::anyhow!(
            "Invalid 4-character encoded value length: {}",
            encoded_val.len()
        ));
    }
    let mut decoded_u32: u32 = 0;
    for (i, c) in encoded_val.chars().enumerate() {
        let val = (c as u32).saturating_sub(0x30); // Subtract ASCII '0'
        if val > 0x3F {
            // Max 6-bit value
            return Err(anyhow::anyhow!("Invalid SCIP2.0 character: {}", c));
        }
        decoded_u32 |= val << (6 * (3 - i)); // Shift according to position (MSB first)
    }
    Ok(decoded_u32)
}

// Helper function to decode a 3-character SCIP2.0 encoded value
pub fn decode_scip_2_0_3char(encoded_val: &str) -> Result<u32, anyhow::Error> {
    if encoded_val.len() != 3 {
        return Err(anyhow::anyhow!(
            "Invalid 3-character encoded value length: {}",
            encoded_val.len()
        ));
    }
    let mut decoded_u32: u32 = 0;
    for (i, c) in encoded_val.chars().enumerate() {
        let val = (c as u32).saturating_sub(0x30); // Subtract ASCII '0'
        if val > 0x3F {
            // Max 6-bit value
            return Err(anyhow::anyhow!("Invalid SCIP2.0 character: {}", c));
        }
        decoded_u32 |= val << (6 * (2 - i)); // Shift according to position (MSB first)
    }
    Ok(decoded_u32)
}
