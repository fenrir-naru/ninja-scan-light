require 'rspec'

require 'tempfile'

$: << File::join(File::dirname(__FILE__), '..', 'build_SWIG')
require 'GPS.so'

describe 'GPS solver' do
  let(:input){{
    :rinex_nav => Tempfile::open{|f|
      if true then
        f.write(<<-__RINEX_NAV_TEXT__)
     2.10           N: GPS NAV DATA                         RINEX VERSION / TYPE
teqc  2013Mar15     BKG Frankfurt       20150617 00:16:14UTCPGM / RUN BY / DATE
Linux 2.4.21-27.ELsmp|Opteron|gcc -static|Linux x86_64|=+   COMMENT
     2.10           N: GPS NAV DATA                         COMMENT
teqc  2008Feb15                         20150617 00:10:22UTCCOMMENT
Linux 2.4.20-8|Pentium IV|gcc -static|Linux|486/DX+         COMMENT
    1.3970D-08  2.2352D-08 -1.1921D-07 -1.1921D-07          ION ALPHA
    1.1059D+05  1.6384D+05 -6.5536D+04 -5.2429D+05          ION BETA
    3.725290298462D-09 1.509903313490D-14   319488     1849 DELTA-UTC: A0,A1,T,W
    16                                                      LEAP SECONDS
Concatenated RINEX files (28/)                              COMMENT
Original file name: brdc1670.15n                            COMMENT
                                                            END OF HEADER
31 15  6 16  0  0  0.0 3.128303214908D-04-1.136868377216D-12 0.000000000000D+00
    7.800000000000D+01 1.292812500000D+02 4.074098273975D-09 1.410690806649D+00
    6.768852472305D-06 8.027532137930D-03 4.271045327187D-06 5.153535821915D+03
    1.728000000000D+05 9.872019290924D-08 8.650894583569D-01 8.940696716309D-08
    9.750288799237D-01 3.013750000000D+02-5.268559212524D-01-8.036049019412D-09
    3.664438352852D-10 1.000000000000D+00 1.849000000000D+03 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-1.350417733192D-08 7.800000000000D+01
    1.663560000000D+05 4.000000000000D+00
24 15  6 16  0  0  0.0-4.998780786991D-05-5.684341886081D-13 0.000000000000D+00
    9.000000000000D+01 1.401250000000D+02 4.642336229081D-09 2.380519254182D+00
    7.258728146553D-06 3.228595596738D-03 3.971159458160D-06 5.153673912048D+03
    1.728000000000D+05-2.980232238770D-08 8.160388274984D-01-1.490116119385D-08
    9.533772739559D-01 2.969375000000D+02 3.612098431884D-01-8.379634759709D-09
    4.025167664390D-10 1.000000000000D+00 1.849000000000D+03 0.000000000000D+00
    2.900000000000D+00 0.000000000000D+00 2.328306436539D-09 9.000000000000D+01
    1.656600000000D+05 4.000000000000D+00
12 15  6 16  0  0  0.0 3.018006682396D-04 3.183231456205D-12 0.000000000000D+00
    9.000000000000D+01-8.984375000000D+01 4.146244136282D-09 1.019955493515D+00
   -4.906207323074D-06 5.596161005087D-03 2.166256308556D-06 5.153664655685D+03
    1.728000000000D+05-3.352761268616D-08 1.926955253672D+00 9.685754776001D-08
    9.895896897770D-01 3.531562500000D+02 6.166481681571D-01-8.222842514396D-09
   -2.525105180766D-10 1.000000000000D+00 1.849000000000D+03 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-1.210719347000D-08 9.000000000000D+01
    1.708800000000D+05 4.000000000000D+00
25 15  6 16  0  0  0.0-2.291053533554D-06-4.206412995700D-12 0.000000000000D+00
    4.300000000000D+01-8.231250000000D+01 4.456614188797D-09 3.514302168106D-01
   -4.393979907036D-06 4.659408819862D-03 1.480802893639D-06 5.153666315079D+03
    1.728000000000D+05 0.000000000000D+00 1.880081553172D+00 1.080334186554D-07
    9.780612252959D-01 3.641250000000D+02 7.406192694225D-01-8.389992700586D-09
   -1.742929689463D-10 1.000000000000D+00 1.849000000000D+03 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00 5.587935447693D-09 4.300000000000D+01
    1.728000000000D+05
18 15  6 15 23 59 44.0 4.096841439605D-04 2.955857780762D-12 0.000000000000D+00
    0.000000000000D+00-2.200000000000D+01 5.964177003340D-09-4.658085121885D-01
   -1.063570380211D-06 1.631921075750D-02 5.243346095085D-06 5.153597631454D+03
    1.727840000000D+05-9.499490261078D-08-1.282443549024D+00 1.769512891769D-07
    9.246030417022D-01 2.601875000000D+02-1.955724914307D+00-8.981445541829D-09
   -4.760912596834D-10 1.000000000000D+00 1.849000000000D+03 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-1.117587089539D-08 0.000000000000D+00
    1.696560000000D+05 4.000000000000D+00
29 15  6 16  0  0  0.0 6.168931722641D-04 2.273736754432D-12 0.000000000000D+00
    7.300000000000D+01-1.041875000000D+02 4.061597753278D-09 5.907941646969D-01
   -5.280598998070D-06 1.195417135023D-03 1.201778650284D-05 5.153755130768D+03
    1.728000000000D+05 9.313225746155D-09 2.981267878597D+00-1.303851604462D-08
    9.739471617229D-01 1.573125000000D+02-5.419126340021D-01-7.659961925303D-09
    2.107230631757D-10 1.000000000000D+00 1.849000000000D+03 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-9.778887033463D-09 7.300000000000D+01
    1.656180000000D+05 4.000000000000D+00
__RINEX_NAV_TEXT__
      else # equivalent
        open(File::join(File::dirname(__FILE__), "..", "..", "test_log", "brdc1670.15n"), 'r'){|io|
          while (str = io.readline)
            f.puts("%-80s"%[str.chomp])
            break if str =~ /END OF HEADER/
          end
          while !io.eof?
            entry = 8.times.collect{io.readline}
            next unless entry[0] =~ /^([ \d]\d) /
            prn = $1.to_i
            next unless entry[6] =~ /(\d\.\d{12})D([+-]\d{2})\s*$/
            iode = ($1.to_f * (10 ** $2.to_i)).to_i
            next unless [[12, 90], [18, 0], [24, 90], [25, 43], [29, 73], [31, 78]] \
                .include?([prn, iode])
            entry.each{|str|
              f.print("%-80s\n"%[str.chomp])
            }
          end
        }
      end
      f.path
    },
    :measurement => proc{
      l1p = GPS::Measurement::L1_PSEUDORANGE
      l1d = GPS::Measurement::L1_RANGE_RATE
      {
        12 => {l1p => 23707858.8131, l1d => -466.953123206},
        18 => {l1p => 25319310.9754, l1d => -453.797772239},
        24 => {l1p => 25683081.3903, l1d => -579.768048832},
        25 => {l1p => 21551698.5927, l1d => -847.86674626},
        29 => {l1p => 23637198.3968, l1d => -1560.30646593},
        31 => {l1p => 23707474.1231, l1d => -1423.67588761},
      }
    }.call,
    :rinex_obs => Tempfile::open{|f|
      if true then
        f.write(<<-__RINEX_OBS_TEXT__)
     2.11           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE
BINEX2RINEX  1.00   GSI, JAPAN          20150617 18:30:58UTCPGM / RUN BY / DATE
0228                                                        MARKER NAME
GSI, JAPAN          GEOSPATIAL INFORMATION AUTHORITY OF JAPAOBSERVER / AGENCY
00000               TRIMBLE NETR9       4.93,26/NOV/2014    REC # / TYPE / VERS
                    TRM59800.80     GSI                     ANT # / TYPE
 -3952590.4754  3360273.8926  3697987.2632                  APPROX POSITION XYZ
        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N
     1     1                                                WAVELENGTH FACT L1/2
     6    L1    C1    L2    P2    S1    S2                  # / TYPES OF OBSERV
  2015     6    16     0     0    0.0000000     GPS         TIME OF FIRST OBS
    30.000                                                  INTERVAL
    16                                                      LEAP SECONDS
smtt on  - smooth time tag (ms jumps in phase and range)    COMMENT
                                                            END OF HEADER
 15  6 16  0  0  0.0000000  0 13R04R14G22G18G25G14R03G12R13G29R23R24 0.000000000
                                G26
 107962537.996 8  20161244.813    83970896.187 7  20161248.340          53.100  
        45.700  
 108292688.703 7  20315410.305    84227597.491 6  20315417.836          44.900  
        38.600  
 120164532.762 7  22866559.250    93634735.08845  22866565.0164         44.800  
        31.9004 
 127657315.100 5  24292417.195    99473356.27642  24292426.4844         33.000  
        16.9004 
 107108718.011 8  20382088.883    83461307.35447  20382098.2114         52.700  
        44.8004 
 106665594.451 8  20297781.844    83116060.16147  20297788.3284         48.500  
        43.6004 
 112247722.253 8  20968815.328    87303792.973 7  20968819.930          48.200  
        43.900  
 119180856.024 7  22679363.352    92868240.86845  22679370.8554         46.000  
        34.4004 
 113219403.456 7  21202362.125    88059583.077 7  21202366.582          43.300  
        42.600  
 116573807.952 8  22183256.359    90836814.38846  22183263.8554         48.000  
        37.2004 
 118068213.681 8  22071607.477    91830908.595 5  22071616.434          48.300  
        34.300  
 122821571.168 6  22968227.047    95527813.562 5  22968231.008          41.000  
        33.400  
 126887720.863 7  24145939.008    98873654.81144  24145951.6954         42.000  
        24.8004 
 15  6 16  0  0 30.0000000  0 13R04R14G22G18G25G14R03G12R13G29R23R24 0.000000000
                                G26
 107953906.562 8  20159632.578    83964182.831 7  20159636.547          52.900  
        45.500  
 108275524.990 7  20312191.531    84214247.928 6  20312198.477          46.000  
        37.900  
 120243117.022 7  22881512.617    93695969.52045  22881519.3444         43.200  
        30.0004 
 127752623.271 5  24310554.234    99547622.25842  24310562.6254         34.300  
        14.8004 
 107148475.172 8  20389654.469    83492286.95147  20389663.6914         53.000  
        44.7004 
 106649633.973 8  20294744.781    83103623.41147  20294751.4144         49.800  
        44.5004 
 112354437.822 8  20988750.797    87386793.909 7  20988756.371          48.000  
        44.200  
 119276901.051 7  22697639.727    92943081.07945  22697647.7704         47.500  
        34.6004 
 113277695.919 7  21213278.297    88104921.616 7  21213283.438          43.800  
        42.700  
 116499708.794 8  22169155.719    90779074.87246  22169163.7704         48.500  
        37.6004 
 118027431.830 8  22063984.773    91799189.430 5  22063991.707          49.300  
        35.100  
 122710898.974 6  22947531.234    95441735.252 5  22947536.609          40.900  
        33.600  
 126789541.785 6  24127256.484    98797151.68444  24127268.9224         41.900  
        25.3004 
 15  6 16  0  1  0.0000000  0 13R04R14G22G18G25G14R03G12R13G29R23R24 0.000000000
                                G26
 107946098.362 8  20158174.969    83958109.762 7  20158178.289          53.200  
        44.900  
 108258590.494 7  20309014.148    84201076.653 6  20309021.145          46.000  
        38.700  
 120321906.821 7  22896506.344    93757364.10145  22896512.2934         44.600  
        30.2004 
 127847866.993 6  24328677.648    99621838.11742  24328687.3954         37.300  
        16.9004 
 107188670.264 8  20397303.531    83523607.79247  20397312.3054         53.700  
        44.3004 
 106634143.554 8  20291797.133    83091552.94947  20291803.6724         49.900  
        44.0004 
 112461605.174 7  21008771.203    87470146.236 7  21008775.176          47.500  
        43.900  
 119373111.958 8  22715948.289    93018050.54445  22715955.9534         48.100  
        34.1004 
 113336078.148 7  21224212.609    88150329.998 7  21224216.957          43.000  
        42.500  
 116425964.067 7  22155122.617    90721611.48746  22155131.1914         46.200  
        37.6004 
 117987330.119 7  22056486.641    91767999.260 5  22056494.809          46.900  
        34.500  
 122600398.977 6  22926865.672    95355790.881 5  22926871.977          39.700  
        32.500  
 126691542.694 6  24108607.188    98720788.84544  24108620.5234         41.700  
        24.8004 
__RINEX_OBS_TEXT__
      else # equivalent
        # GEONET Setagaya from https://terras.gsi.go.jp/data_service.php#11/35.663712/139.630394
        open(File::join(File::dirname(__FILE__), "..", "..", "test_log", "02281670.15o"), 'r'){|io|
          99.times{|i|
            break unless str = io.readline
            f.write(str)
          }
        }
      end
      f.path
    },
  }}
  let(:solver){
    res = GPS::Solver::new
    res.correction = {:gps_ionospheric => :klobuchar, :gps_tropospheric => :hopfield}
    res
  }
  
  describe 'demo' do
    it 'calculates position without any error' do
      sn = solver.gps_space_node
      puts "RINEX NAV read: %d items."%[sn.read(input[:rinex_nav])]
      meas = GPS::Measurement::new
      input[:measurement].each{|prn, items|
        items.each{|k, v|
          meas.add(prn, k, v)
        }
      }
      expect(meas.to_hash).to eq(proc{|array|
            res = {}
            array.each{|prn, k, v|
              (res[prn][k] = v) rescue (res[prn] = {k => v})
            }
            res
          }.call(meas.to_a))
      expect(GPS::Measurement::new(meas.to_a).to_a.sort).to eq(meas.to_a.sort) # accept [[prn, k, v], ...]
      expect(GPS::Measurement::new(meas.to_hash).to_a.sort).to eq(meas.to_a.sort) # accept {prn => {k => v, ...}, ...}
      expect{GPS::Measurement::new({:sym => {1 => 2}})}.to raise_error
      expect{GPS::Measurement::new({1 => {:sym => 2}})}.to raise_error
      expect{GPS::Measurement::new({1 => [2, 3]})}.to raise_error
      
      t_meas = GPS::Time::new(1849, 172413)
      puts "Measurement time: #{t_meas.to_a} (a.k.a #{"%d/%d/%d %02d:%02d:%02d UTC"%[*t_meas.c_tm]})"
      expect(t_meas.c_tm).to eq([2015, 6, 15, 23, 53, 33])
      
      sn.update_all_ephemeris(t_meas)
      
      [:alpha, :beta].each{|k|
        puts "Iono #{k}: #{sn.iono_utc.send(k)}"
      }
      
      meas.each{|prn, k, v|
        eph = sn.ephemeris(prn)
        puts "XYZ(PRN:#{prn}): #{eph.constellation(t_meas)[0].to_a} (iodc: #{eph.iodc}, iode: #{eph.iode})"
      }
      
      run_solver = proc{
        pvt = solver.solve(meas, t_meas)
        [
          :error_code,
          :position_solved?,
          [:receiver_time, proc{|v| v.to_a}],
          :used_satellite_list,
          [:llh, proc{|llh| llh.to_a.zip([180.0 / Math::PI] * 2 + [1]).collect{|v, sf| v * sf}}],
          :receiver_error,
          [:velocity, proc{|xyz| xyz.to_a}],
          :receiver_error_rate,
          [:G_enu, proc{|mat| mat.to_s}],
          :fd,
          :fde_min,
          :fde_2nd,
        ].each{|fun, task|
          task ||= proc{|v| v}
          puts "pvt.#{fun}: #{task.call(pvt.send(fun))}"
        }
        pvt
      }

      puts "Normal solution ..."
      pvt = run_solver.call
      puts

      expect(pvt.position_solved?).to be(true)
      expect(pvt.receiver_time.to_a).to eq([1849, 172413])
      expect(pvt.llh.to_a).to eq([:lat, :lng, :alt].collect{|k| pvt.llh.send(k)})
      expect(pvt.llh.lat / Math::PI * 180).to be_within(1E-9).of(35.6992591268) # latitude
      expect(pvt.llh.lng / Math::PI * 180).to be_within(1E-9).of(139.541502292) # longitude
      expect(pvt.llh.alt)                 .to be_within(1E-4).of(104.279402455) # altitude
      expect(pvt.receiver_error).to be_within(1E-4).of(1259087.83603)
      expect(pvt.gdop).to be_within(1E-10).of(3.83282723293)
      expect(pvt.pdop).to be_within(1E-10).of(3.30873220653)
      expect(pvt.hdop).to be_within(1E-10).of(2.05428293774)
      expect(pvt.vdop).to be_within(1E-10).of(2.59376761222)
      expect(pvt.tdop).to be_within(1E-10).of(1.9346461648)
      expect(pvt.velocity.to_a).to eq([:e, :n, :u].collect{|k| pvt.velocity.send(k)})
      expect(pvt.velocity.north).to be_within(1E-7).of(-0.839546227836) # north
      expect(pvt.velocity.east) .to be_within(1E-7).of(-1.05805616381)  # east
      expect(pvt.velocity.down) .to be_within(1E-7).of(-0.12355474006)  # down
      expect(pvt.receiver_error_rate).to be_within(1E-7).of(-1061.92654151)
      expect(pvt.G.rows).to eq(6)
      expect(pvt.W.rows).to eq(6)
      expect(pvt.delta_r.rows).to eq(6)
      expect(pvt.G_enu.rows).to eq(6)
      expect(Math::sqrt(pvt.C[3, 3])).to be_within(1E-10).of(pvt.tdop)
      expect(Math::sqrt(pvt.C_enu[2, 2])).to be_within(1E-10).of(pvt.vdop)
      pvt.S.to_a.flatten.zip(
          ((pvt.G.t * pvt.W * pvt.G).inv * (pvt.G.t * pvt.W)).to_a.flatten).each{|a, b|
        expect(a).to be_within(1E-10).of(b)
      }
      pvt.S_enu.to_a.flatten.zip(
          ((pvt.G_enu.t * pvt.W * pvt.G_enu).inv * (pvt.G_enu.t * pvt.W)).to_a.flatten).each{|a, b|
        expect(a).to be_within(1E-10).of(b)
      }
      expect([:rows, :columns].collect{|f| pvt.slope_HV_enu.send(f)}).to eq([6, 2])
      expect(pvt.used_satellites).to eq(6)
      expect(pvt.used_satellite_list).to eq([12,18, 24, 25, 29, 31])

      meas.each{|prn, k, v|
        solver.gps_options.exclude(prn)
        puts "Excluded(PRN: #{solver.gps_options.excluded.join(', ')}) solution ..."
        run_solver.call
        solver.gps_options.excluded.each{|prn|
          solver.gps_options.include(prn)
        }
        puts
      }
    end
    
    it 'can be modified through hooks' do
      sn = solver.gps_space_node
      expect(solver.correction[:gps_ionospheric]).to include(:klobuchar)
      expect(solver.correction[:gps_tropospheric]).to include(:hopfield)
      expect{solver.correction = nil}.to raise_error
      expect{solver.correction = {
        :gps_ionospheric => [:klobuchar, :no_correction],
        :options => {:f_10_7 => 10},
      }}.not_to raise_error
      expect(solver.correction[:gps_ionospheric]).to include(:no_correction)
      expect(solver.correction[:options][:f_10_7]).to eq(10)
      sn.read(input[:rinex_nav])
      t_meas = GPS::Time::new(1849, 172413)
      sn.update_all_ephemeris(t_meas)
      solver.hooks[:relative_property] = proc{|prn, rel_prop, meas, rcv_e, t_arv, usr_pos, usr_vel|
        expect(input[:measurement]).to include(prn)
        expect(meas).to be_a_kind_of(Hash)
        expect(t_arv).to be_a_kind_of(GPS::Time)
        expect(usr_pos).to be_a_kind_of(Coordinate::XYZ)
        expect(usr_vel).to be_a_kind_of(Coordinate::XYZ)
        weight, range_c, range_r, rate_rel_neg, *los_neg = rel_prop
        weight = 1
        [weight, range_c, range_r, rate_rel_neg] + los_neg
      }
      solver.hooks[:update_position_solution] = proc{|*mats|
        mats.each{|mat|
          expect(mat).to be_a_kind_of(SylphideMath::MatrixD)
        }
        mat_G, mat_W, mat_delta_r = mats
      }
      solver.hooks[:satellite_position] = proc{
        i = 0
        proc{|prn, time, pos|
          expect(input[:measurement]).to include(prn)
          expect(pos).to be_a_kind_of(Coordinate::XYZ).or eq(nil)
          # System_XYZ or [x,y,z] or nil(= unknown position) are acceptable
          case (i += 1) % 5
          when 0
            nil
          when 1
            pos.to_a
          else
            pos
          end
        }
      }.call
      pvt = solver.solve(
          input[:measurement].collect{|prn, items|
            items.collect{|k, v| [prn, k, v]}
          }.flatten(1),
          t_meas)
      expect(pvt.W).to eq(SylphideMath::MatrixD::I(pvt.W.rows))
    end
    
    it 'calculates position without any error with RINEX obs file' do
      sn = solver.gps_space_node
      puts "RINEX NAV read: %d items."%[sn.read(input[:rinex_nav])]
      GPS::RINEX_Observation::read(input[:rinex_obs]){|item|
        t_meas = item[:time]
        puts "Measurement time: #{t_meas.to_a} (a.k.a #{"%d/%d/%d %02d:%02d:%02d UTC"%[*t_meas.c_tm]})"
        sn.update_all_ephemeris(t_meas)
        
        meas = GPS::Measurement::new
        types = (item[:meas_types]['G'] || item[:meas_types][' ']).collect.with_index{|type_, i|
          case type_
          when "C1"
            [i, GPS::Measurement::L1_PSEUDORANGE]
          when "D1"
            [i, GPS::Measurement::L1_RANGE_RATE]
          else
            nil 
          end
        }.compact
        item[:meas].each{|k, v|
          sys, prn = k
          next unless sys == 'G' # GPS only
          types.each{|i, type_|
            meas.add(prn, type_, v[i][0]) if v[i]
          }
        }

        pvt = solver.solve(meas, t_meas)
        expect(pvt.position_solved?).to eq(true)

        puts pvt.llh.to_a.zip([180.0 / Math::PI] * 2 + [1]).collect{|v, sf| v * sf}.inspect
        if approx_pos = proc{
          next false unless res = item[:header].select{|k, v| k =~ /APPROX POSITION XYZ/}.values[0]
          next false unless res = res.collect{|item|
            item.scan(/([+-]?\d+(?:\.\d+)?)\s*/).flatten
          }.reject{|item|
            item.empty?
          }[0]
          Coordinate::XYZ::new(*(res.collect{|str| str.to_f}))
        }.call then
          approx_pos.to_a.zip(pvt.xyz.to_a).each{|a, b|
            expect(a).to be_within(1E+1).of(b) # 10 m
          } 
        end

        pvt.used_satellite_list.each{|prn|
          eph = sn.ephemeris(prn)
          puts "XYZ(PRN:#{prn}): #{eph.constellation(t_meas)[0].to_a} (iodc: #{eph.iodc}, iode: #{eph.iode})"
        }
      }
    end
  end
end