#!/usr/bin/ruby

=begin
This is a generator of renew.cfg, which is used to change configuration of NinjaScan-Light.
=end

class C_NUMBER
  def C_NUMBER.to_str(v)
    v.inspect
  end
end

class U8 < C_NUMBER
  TYPENAME = :u8
  def U8.pack(v)
    [v].pack('C')
  end
end

class U16 < C_NUMBER
  TYPENAME = :u16
  def U16.pack(v)
    [v].pack('v')
  end
end

class U32 < C_NUMBER
  TYPENAME = :u32
  def U32.pack(v)
    [v].pack('V')
  end
end

class UBX_CFG
  TYPENAME = :ubx_cfg_t
  def UBX_CFG.to_str(v)
    sprintf("{0x%02X, 0x%02X, %d}", *v) 
  end
  def UBX_CFG.pack(v)
    (v || [0, 0, 0]).pack('C3')
  end
end

class C_ARRAY
  attr_accessor :type, :size
  def initialize(type, size)
    @type = type
    @size = size
  end
end

class String
  def pretty(level = 0)
    '  ' * level + self + $/
  end
  
end

def get_struct(elm, level = 0, res = "")
  case elm[1]
  when Array
    res += "struct {".pretty(level)
    elm[1..-1].each{|child|
      res = get_struct(child, level + 1, res)
    }
    res += "} #{elm[0]};".pretty(level)
  when C_ARRAY
    res += "#{elm[1].type::TYPENAME} #{elm[0]}[#{elm[1].size}];".pretty(level)
  else
    res += "#{elm[1]::TYPENAME} #{elm[0]};".pretty(level)
  end
  res.chomp!.slice!(-1) if level == 0
  return res
end

def get_value(elm, opt = {:verbose => true}, level = 0, res = "")
  case elm[1]
  when Array
    res += "{#{" // #{elm[0]}" if opt[:verbose]}".pretty(level)
    elm[1..-1].each{|child|
      res = get_value(child, opt, level + 1, res)
    }
    res += "},".pretty(level)
  when C_ARRAY
    res += "{#{" // #{elm[0]}" if opt[:verbose]}".pretty(level)
    elm[1].size.times{|i|
      v = elm[2 + i]
      next unless v
      res += "#{elm[1].type.to_str(v)},".pretty(level + 1)
    }
    res += "},".pretty(level)
  else
    res += "#{elm[1].to_str(elm[2])},#{" // #{elm[0]}" if opt[:verbose]}".pretty(level)
  end
  res.chomp!.slice!(-1) if level == 0
  return res
end

def get_binary(elm, res = "")
  case elm[1]
  when Array
    elm[1..-1].each{|child|
      res = get_binary(child, res)
    }   
  when C_ARRAY
    elm[1].size.times{|i|
      res += elm[1].type.pack(elm[2 + i])
    }
  else
    res += elm[1].pack(elm[2])
  end
  return res
end

config = [:config, # The following is the default, please customize it!
  [:baudrate,
    [:gps, U32, 115200], # GPS baudrate [bps]
    [:telemeter, U32, 9600], # UART1 device (called as telemeter) baudrate [bps]
  ],
  [:gps,
    [:rate,
      [:measurement_ms, U16, 200], # GPS update rate [ms]
      [:navigation_cycles, U16, 1], # GPS navigation update rate; 1 [s] / (200 [ms] * 1 [cycle]) = 5 Hz
    ],
    [:message, C_ARRAY::new(UBX_CFG, 16), # GPS output selection, maximum 16 slots, currently 10 slots have been occupied. 
      [0x01, 0x02, 1],  # NAV-POSLLH   // 28 + 8 = 36 bytes
      [0x01, 0x03, 5],  # NAV-STATUS   // 16 + 8 = 24 bytes
      [0x01, 0x04, 5],  # NAV-DOP      // 18 + 8 = 26 bytes
      [0x01, 0x06, 1],  # NAV-SOL      // 52 + 8 = 60 bytes
      [0x01, 0x12, 1],  # NAV-VELNED   // 36 + 8 = 44 bytes
      [0x01, 0x20, 20], # NAV-TIMEGPS  // 16 + 8 = 24 bytes
      [0x01, 0x21, 20], # NAV-TIMEUTC  // 20 + 8 = 28 bytes
      [0x01, 0x30, 10], # NAV-SVINFO   // (8 + 12 * x) + 8 = 112 bytes (@ 8 satellites)
      [0x02, 0x10, 1],  # RXM-RAW      // (8 + 24 * x) + 8 = 208 bytes (@ 8 satellites)
      [0x02, 0x11, 1],  # RXM-SFRB     // 42 + 8 = 50 bytes
    ],
  ],
  [:inertial,
    [:gyro_config, U8, (3 << 3)], # (3 << 3) is full scale 2000 [dps]
    [:accel_config, U8, (2 << 3)], # (2 << 3) is full scale 8 [G]
  ],
  [:telemetry_truncate,
    [:a_page, U8, 20], # a_page telemetry is approximate 5 Hz
    [:p_page, U8, 2], # a_page telemetry is approximate 1 Hz
    [:m_page, U8, 2], # a_page telemetry is approximate 1 Hz
    [:g_page,
      [:item, C_ARRAY::new(UBX_CFG, 4), # g_page telemetry selection, maximum 4 slots, currently 1 slot has been occupied.
        [0x01, 0x06, 5], # NAV-SOL: approximately 1 Hz]
      ]
    ]
  ]
]

$stderr.puts "Generate #{get_struct(config)} = #{get_value(config)};"

OUTPUT_FNAME = 'renew.cfg'
data = get_binary(config)
$stderr.puts "Debug output: #{data.unpack("C*").collect{|v| sprintf('0x%02X', v)}.join(' ')}"
open(OUTPUT_FNAME, 'wb'){|io| io.print data}
$stderr.puts "File(#{OUTPUT_FNAME}) saved."