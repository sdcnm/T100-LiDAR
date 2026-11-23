-- T100 UDP field decoder. CRC fields are exposed, not validated here.
-- Load with: tshark -X lua_script:extras/wireshark/t100.lua -r capture.pcap
local t100 = Proto("t100", "T100 LiDAR")
local f = {
  length = ProtoField.uint16("t100.length", "Length", base.DEC),
  sender = ProtoField.uint8("t100.sender", "Sender", base.HEX),
  receiver = ProtoField.uint8("t100.receiver", "Receiver", base.HEX),
  sequence = ProtoField.uint16("t100.sequence", "Sequence", base.DEC),
  flags = ProtoField.uint8("t100.flags", "Flags", base.HEX),
  command_set = ProtoField.uint8("t100.command_set", "Command set", base.HEX),
  command_id = ProtoField.uint8("t100.command_id", "Command ID", base.HEX),
  crc8 = ProtoField.uint8("t100.crc8", "CRC8 (not verified)", base.HEX),
  crc16 = ProtoField.uint16("t100.crc16", "CRC16 (not verified)", base.HEX),
  crc32 = ProtoField.uint32("t100.crc32", "CRC32 (not verified)", base.HEX),
  payload = ProtoField.bytes("t100.payload", "Payload"),
  dtype = ProtoField.uint8("t100.data_type", "Data type", base.DEC, {[0]="IMU",[1]="XYZ points"}),
  count = ProtoField.uint16("t100.count", "Record count", base.DEC),
  time_type = ProtoField.uint8("t100.time_type", "Time type", base.DEC),
  timestamp = ProtoField.uint64("t100.timestamp_ns", "Timestamp (ns)", base.DEC),
  x = ProtoField.int32("t100.x_mm", "X (mm)", base.DEC),
  y = ProtoField.int32("t100.y_mm", "Y (mm)", base.DEC),
  z = ProtoField.int32("t100.z_mm", "Z (mm)", base.DEC),
  reflectivity = ProtoField.uint8("t100.reflectivity", "Reflectivity", base.DEC),
  tag = ProtoField.uint8("t100.tag", "Tag", base.HEX),
  gyro_x = ProtoField.float("t100.gyro_x", "Gyro X (rad/s)"),
  gyro_y = ProtoField.float("t100.gyro_y", "Gyro Y (rad/s)"),
  gyro_z = ProtoField.float("t100.gyro_z", "Gyro Z (rad/s)"),
  acc_x = ProtoField.float("t100.acc_x", "Acceleration X (g)"),
  acc_y = ProtoField.float("t100.acc_y", "Acceleration Y (g)"),
  acc_z = ProtoField.float("t100.acc_z", "Acceleration Z (g)"),
  key = ProtoField.uint16("t100.status_key", "Status key", base.HEX),
  item_length = ProtoField.uint16("t100.item_length", "Item length", base.DEC)
}
t100.fields = f

function t100.dissector(buffer, pinfo, tree)
  local size = buffer:len()
  if size >= 13 and buffer(0,1):uint() == 0x55 then
    local declared = buffer(1,1):uint() + (buffer(2,1):uint() % 4) * 256
    if declared ~= size or math.floor(buffer(2,1):uint() / 4) ~= 1 then return 0 end
    pinfo.cols.protocol = "T100"
    local cs, ci = buffer(9,1):uint(), buffer(10,1):uint()
    pinfo.cols.info = string.format("Control %02X/%02X seq=%d (%d bytes)",cs,ci,buffer(6,2):le_uint(),size)
    local node = tree:add(t100, buffer())
    node:add(f.length, buffer(1,2), declared)
    node:add(f.crc8, buffer(3,1))
    node:add(f.sender, buffer(4,1)); node:add(f.receiver, buffer(5,1))
    node:add_le(f.sequence, buffer(6,2)); node:add(f.flags, buffer(8,1))
    node:add(f.command_set, buffer(9,1)); node:add(f.command_id, buffer(10,1))
    node:add_le(f.crc16, buffer(size-2,2))
    if size > 13 then node:add(f.payload, buffer(11,size-13)) end
    if cs == 1 and ci == 2 and size >= 17 then
      local count, offset = buffer(11,2):le_uint(), 15
      for _ = 1, count do
        if offset + 4 > size - 2 then break end
        local n = buffer(offset+2,2):le_uint()
        if offset + 4 + n > size - 2 then break end
        local item = node:add(t100,buffer(offset,4+n),"Status item")
        item:add_le(f.key,buffer(offset,2)); item:add_le(f.item_length,buffer(offset+2,2))
        if n > 0 then item:add(f.payload,buffer(offset+4,n)) end
        offset = offset + 4 + n
      end
    end
    return size
  end
  if size < 36 or buffer(0,1):uint() ~= 1 or buffer(1,2):le_uint() ~= size then return 0 end
  local dtype, count = buffer(10,1):uint(), buffer(5,2):le_uint()
  if not ((dtype == 1 and size == 1380 and count == 96) or (dtype == 0 and size == 60 and count == 1)) then return 0 end
  pinfo.cols.protocol = "T100"
  pinfo.cols.info = string.format("%s seq=%d records=%d",dtype==1 and "Point cloud" or "IMU",buffer(7,2):le_uint(),count)
  local node = tree:add(t100,buffer())
  node:add_le(f.length,buffer(1,2)); node:add_le(f.count,buffer(5,2))
  node:add_le(f.sequence,buffer(7,2)); node:add(f.dtype,buffer(10,1))
  node:add(f.time_type,buffer(11,1)); node:add_le(f.crc32,buffer(24,4))
  node:add_le(f.timestamp,buffer(28,8))
  if dtype == 1 then
    for i = 0, count - 1 do
      local off = 36 + i * 14
      local point = node:add(t100,buffer(off,14),"Point " .. i)
      point:add_le(f.x,buffer(off,4)); point:add_le(f.y,buffer(off+4,4)); point:add_le(f.z,buffer(off+8,4))
      point:add(f.reflectivity,buffer(off+12,1)); point:add(f.tag,buffer(off+13,1))
    end
  else
    node:add_le(f.gyro_x,buffer(36,4)); node:add_le(f.gyro_y,buffer(40,4)); node:add_le(f.gyro_z,buffer(44,4))
    node:add_le(f.acc_x,buffer(48,4)); node:add_le(f.acc_y,buffer(52,4)); node:add_le(f.acc_z,buffer(56,4))
  end
  return size
end

local ports = DissectorTable.get("udp.port")
for _, port in ipairs({50000,60000,60001,60003,60010}) do ports:add(port,t100) end
ports:add_for_decode_as(t100)
