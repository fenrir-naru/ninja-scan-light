#!/usr/bin/ruby

=begin
This is a generator of renew.cfg, which is used to change configuration of NinjaScan-Light.
=end

class U8
  TYPENAME = :u8
  def U8.pack(v)
    [v].pack('C')
  end
end

class U16
  TYPENAME = :u16
  def U16.pack(v)
    [v].pack('v')
  end
end

class U32
  TYPENAME = :u32
  def U32.pack(v)
    [v].pack('V')
  end
end

class UBX_CFG
  TYPENAME = :ubx_cfg_t
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

def get_struct(elm, level = 0, res = "")
  case elm[1]
  when Array
    res += "#{'  ' * level}struct {#{$/}"
    elm[1..-1].each{|child|
      res = get_struct(child, level + 1, res)
    }
    res += "#{'  ' * level}} #{elm[0]};#{$/}"
  when C_ARRAY
    res += "#{'  ' * level}#{elm[1].type::TYPENAME} #{elm[0]}[#{elm[1].size}];#{$/}"
  else
    res += "#{'  ' * level}#{elm[1]::TYPENAME} #{elm[0]};#{$/}"
  end
  return res
end

def get_value(elm, res = "")
  case elm[1]
  when Array
    elm[1..-1].each{|child|
      res = get_value(child, res)
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

config = [:config,
  [:baudrate,
    [:gps, U32, 115200],
    [:telemeter, U32, 9600],
  ],
  [:gps,
    [:rate,
      [:measurement_ms, U16, 200],
      [:navigation_cycles, U16, 1],
    ],
    [:message, C_ARRAY::new(UBX_CFG, 16), 
      [0x01, 0x02, 1],  # NAV-POSLLH   // 28 + 8 = 36 bytes
      [0x01, 0x03, 5],  # NAV-STATUS   // 16 + 8 = 24 bytes
      [0x01, 0x04, 5],  # NAV-DOP      // 18 + 8 = 26 bytes
      [0x01, 0x06, 1],  # NAV-SOL      // 52 + 8 = 60 bytes
      [0x01, 0x12, 1],  # NAV-VELNED   // 36 + 8 = 44 bytes
      [0x01, 0x20, 20], # NAV-TIMEGPS  // 16 + 8 = 24 bytes
      [0x01, 0x21, 20], # NAV-TIMEUTC  // 20 + 8 = 28 bytes
      [0x01, 0x30, 10], # NAV-SVINFO   // (8 + 12 * x) + 8 = 112 bytes (@8)
      [0x02, 0x10, 1],  # RXM-RAW      // (8 + 24 * x) + 8 = 208 bytes (@8)
      [0x02, 0x11, 1],  # RXM-SFRB     // 42 + 8 = 50 bytes
    ],
  ],
  [:inertial,
    [:gyro_config, U8, (3 << 3)],
    [:accel_config, U8, (2 << 3)],
  ],
  [:telemetry_truncate,
    [:a_page, U8, 20],
    [:p_page, U8, 2],
    [:m_page, U8, 2],
    [:g_page,
      [:item, C_ARRAY::new(UBX_CFG, 4),  
        [0x01, 0x06, 5], # NAV-SOL: approximately 1 Hz]
      ]
    ]
  ]
]

$stderr.puts "Generate #{get_struct(config)}"

OUTPUT_FNAME = 'renew.cfg'
data = get_value(config)
$stderr.puts "Debug output: #{data.unpack("C*").collect{|v| sprintf('0x%02X', v)}.join(' ')}"
open(OUTPUT_FNAME, 'w'){|io| io.print data}
$stderr.puts "File(#{OUTPUT_FNAME}) saved."