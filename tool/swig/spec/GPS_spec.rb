require 'rspec'

require 'tempfile'

$: << File::join(File::dirname(__FILE__), '..', 'build_SWIG')
require 'GPS.so'

describe 'GPS solver' do
  let(:input){{
    :rinex => Tempfile::open{|f|
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
        open(File::join(File::dirname(__FILE__), "..", "..", "test_log", "brdc1670.15n")){|io|
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
    }.call
  }}
  let(:solver){GPS::Solver::new}
  let(:meas){GPS::Measurement::new}
  
  describe 'demo' do
    it 'calculates position without any error' do
      sn = solver.gps_space_node
      puts "RINEX NAV read: %d items."%[sn.read(input[:rinex])]
      input[:measurement].each{|prn, items|
        items.each{|k, v|
          meas.add(prn, k, v)
        }
      }
      
      t_meas = GPS::Time::new(1849, 172413)
      puts "Measurement time: #{t_meas.to_a} (a.k.a #{"%d/%d/%d %d:%d:%d UTC"%[*t_meas.c_tm]})"
      expect(t_meas.c_tm).to eq([2015, 6, 15, 23, 53, 33])
      
      sn.update_all_ephemeris(t_meas)
      
      [:alpha, :beta].each{|k|
        puts "Iono #{k}: #{sn.iono_utc.send(k)}"
      }
      puts solver.gps_options.ionospheric_models
      
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
          :used_satellites,
          [:llh, proc{|llh| llh.to_a.zip([180.0 / Math::PI] * 2 + [1]).collect{|v, sf| v * sf}}],
          :receiver_error,
          [:velocity, proc{|xyz| xyz.to_a}],
          :receiver_error_rate,
          [:G, proc{|mat| mat.to_s}],
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
      expect(pvt.llh.to_a[0] / Math::PI * 180).to be_within(1E-9).of(35.6992591268) # latitude
      expect(pvt.llh.to_a[1] / Math::PI * 180).to be_within(1E-9).of(139.541502292) # longitude
      expect(pvt.llh.to_a[2])                 .to be_within(1E-4).of(104.279402455) # altitude
      expect(pvt.receiver_error).to be_within(1E-4).of(1259087.83603)
      expect(pvt.gdop).to be_within(1E-10).of(3.83282723293)
      expect(pvt.pdop).to be_within(1E-10).of(3.30873220653)
      expect(pvt.hdop).to be_within(1E-10).of(2.56304399953)
      expect(pvt.vdop).to be_within(1E-10).of(2.09248996915)
      expect(pvt.tdop).to be_within(1E-10).of(1.9346461648)
      expect( pvt.velocity.to_a[1]).to be_within(1E-7).of(-0.839546227836) # north
      expect( pvt.velocity.to_a[0]).to be_within(1E-7).of(-1.05805616381)  # east
      expect(-pvt.velocity.to_a[2]).to be_within(1E-7).of(-0.12355474006)  # down
      expect(pvt.receiver_error_rate).to be_within(1E-7).of(-1061.92654151)

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
  end
end