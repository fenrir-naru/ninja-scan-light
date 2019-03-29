#!/usr/bin/ruby
#coding cp932

class R_Bootstrap
  
  SETUP_INF = <<-__TEXT__
[Setup]
Dir=DST_DIR
Group=R
NoIcons=1
SetupType=custom
Components=main,x64,translations
Tasks=
[R]
MDISDI=MDI
HelpStyle=HTML
  __TEXT__
  
  DEFAULT_PROP = {
    :dst_dir => File::join(File::dirname(File::absolute_path(__FILE__)), 'R'),
    :url => 'https://cran.ism.ac.jp/bin/windows/base/R-3.5.3-win.exe',
  }
  
  class <<self
    def setup(opt = {})
      opt = DEFAULT_PROP.merge(opt)
      require 'open-uri'
      
      $stderr.print "1) Downloading R "
      open(opt[:url], 'rb',
          proc{
            len = nil
            step_bytes = 1_000_000
            steps = 0
            {
              :content_length_proc => proc{|content_length|
                len = content_length
                step_bytes = len / 100
              },
              :progress_proc => proc{|transferred_bytes|
                next_steps = (transferred_bytes / step_bytes).to_i
                (steps...next_steps).each{|i|
                  $stderr.print (i % 10 == 9 ? "#{i+1}#{(len ? "%" : "M")}" : ".")
                }
                steps = next_steps
              },
            }
          }.call){|io|
        $stderr.puts " done."
        
        require 'tempfile'
        installer = Tempfile::open(['R-install', '.exe']){|io2|
          io2.binmode
          io2.write(io.read)
          io2.path
        }
        setup_conf = Tempfile::open(['R-setup', '.inf']){|io2|
          io2.binmode
          io2.print(SETUP_INF.gsub('DST_DIR', opt[:dst_dir]).gsub(/\R/, "\r\n"))
          io2.path
        } 
        FileUtils::chmod(0755, installer)
        cmd = "#{installer} /LOADINF=\"#{setup_conf}\" /SILENT"
        $stderr.print "2) Deploying 64-bit version R (#{cmd}) ..."
        system(cmd)
        $stderr.puts " done."
      }
    end
  end
end

if $0 == __FILE__ then
  R_Bootstrap::setup
  
  require 'rinruby_without_r_constant'
  r = RinRuby::new({
    :executable => File::join(R_Bootstrap::DEFAULT_PROP[:dst_dir], 'bin', 'x64', 'Rterm.exe'),
  })
  p r
  r.prompt
end
