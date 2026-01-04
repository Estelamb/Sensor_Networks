-- Helper function to convert bytes to Integer (Little Endian)
function bytesToInt(bytes, start, size, signed)
    local val = 0
    for i = 0, size - 1 do
        -- Lua arrays are 1-based index
        val = val + (bytes[start + i] * (256 ^ i))
    end
    if signed and val >= (256 ^ size) / 2 then
        val = val - (256 ^ size)
    end
    return val
end

function parsePayload(appeui, deveui, payload)
    -- Convert hex payload to byte array
    local bytes = resiot_hexdecode(payload)

    -- 1. Light (2 bytes) - Pos 1-2 (uint16)
    local light = bytesToInt(bytes, 1, 2, false) / 10.0
    
    -- 2. Latitude (4 bytes) - Pos 3-6 (int32)
    local lat = bytesToInt(bytes, 3, 4, true) / 1000000.0
    
    -- 3. Longitude (4 bytes) - Pos 7-10 (int32)
    local lon = bytesToInt(bytes, 7, 4, true) / 1000000.0

	-- 4. Altitude (4 bytes) - Pos 11-14 (int32)
    local alt = bytesToInt(bytes, 11, 4, true) / 100.0
    
    -- 5. Temperature (2 bytes) - Pos 15-16 (int16)
    local temp = bytesToInt(bytes, 15, 2, true) / 100.0
    
    -- 6. Humidity (2 bytes) - Pos 17-18 (uint16)
    local hum = bytesToInt(bytes, 17, 2, false) / 100.0

    -- Debug Logs
    resiot_debug(string.format("Decoded -> Light: %.1f, Lat: %.6f, Lon: %.6f, Temp: %.2f, Hum: %.2f", 
                 light, lat, lon, temp, hum))

    -- Update Nodes in ResIoT
    resiot_setnodevalue(appeui, deveui, "Light", light)
    resiot_setnodevalue(appeui, deveui, "Latitude", lat)
    resiot_setnodevalue(appeui, deveui, "Longitude", lon)
    resiot_setnodevalue(appeui, deveui, "Temperature", temp)
    resiot_setnodevalue(appeui, deveui, "Humidity", hum)
end

-- --- Main Process ---
Origin = resiot_startfrom()

if Origin == "Manual" then
    -- Test payload (14 bytes hex)
    payload = "0A00FE3E2102996E2E08E808E813" 
    appeui = "70b3d57ed000fc4d"
    deveui = "7a39323559379194"
else
    appeui = resiot_comm_getparam("appeui")
    deveui = resiot_comm_getparam("deveui")
    payload, err = resiot_getlastpayload(appeui, deveui)
end

parsePayload(appeui, deveui, payload)