#!/usr/bin/ruby

class R_Bootstrap
  class <<self
    def registry_check
      [:HKLM, :HKCU].collect{|head|
        `reg query "#{head}\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall" /f "R for Windows*is1" 2>nul`.scrub.lines
      }.flatten.collect{|line|
        next unless line.strip =~ /^\s*(.*_is1)/
        $1
      }.compact
    end
    
    def setup_options(opt = {})
      r_ver = opt[:ver] || '3.5.3'
      dst_dir = opt[:dst_dir] \
          || File::join(File::dirname(File::absolute_path(__FILE__)), "R-#{r_ver}")
      arch = opt[:arch] || case `echo %PROCESSOR_ARCHITECTURE% `
        when /AMD64/; 'x64'
        when /x86/; `echo %PROCESSOR_ARCHITEW6432% ` =~ /AMD86/ ? 'x64' : 'i386'
        else; 'i386'
      end
      {
        :dst_dir => dst_dir,
        :r_ver => r_ver,
        :arch => arch,
        :src_url => "https://cran.ism.ac.jp/bin/windows/base/R-#{r_ver}-win.exe",
        :setup_inf => (<<-__TEXT__).gsub(/\R/, "\r\n"),
[Setup]
Dir=#{dst_dir}
Group=R
NoIcons=1
SetupType=custom
Components=main,#{arch},translations
Tasks=
[R]
MDISDI=MDI
HelpStyle=HTML
            __TEXT__
      }.merge(opt)
    end
    
    def check_installation(opt = {})
      opt = setup_options(opt)
      File::exist?(File::join(opt[:dst_dir], 'bin', opt[:arch], 'R.exe')) ? opt : nil
    end
    
    def setup(opt = {})
      opt = setup_options(opt)
      
      $stderr.print "0) Check previous installation of R #{opt[:r_ver]}: "
      checked = check_installation(opt)
      if checked then
        $stderr.puts "found, installation will be skipped."
        return checked
      else
        $stderr.puts "not found."
      end 
        
      require 'open-uri'
      require 'tempfile'
      
      $stderr.print "1) Downloading R #{opt[:r_ver]} installer"
      installer = open(opt[:src_url], 'rb',
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
        
        Tempfile::open(['R-install', '.exe']){|io2|
          io2.binmode
          io2.write(io.read)
          io2.path
        }
      }
      FileUtils::chmod(0755, installer)
      
      setup_conf = Tempfile::open(['R-setup', '.inf']){|io|
        io.binmode
        io.print(opt[:setup_inf])
        io.path
      }
      
      # check uninstaller registry entries before install
      reg_before = registry_check()    
      
      cmd = "#{installer} /LOADINF=\"#{setup_conf}\" /SILENT"
      $stderr.print "2) Deploying R (#{cmd}) ..."
      system(cmd)
      $stderr.puts " done."
      
      # Remove uninstaller entry
      (registry_check() - reg_before).each{|key|
        `reg delete "#{key}" /f`
      }
      
      check_installation(opt)
    end
  end
end

R_Bootstrap::setup if $0 == __FILE__

__END__
require 'rinruby_without_r_constant'
r = RinRuby::new({
  :executable => File::join(r_dir, 'bin', 'x64', 'Rterm.exe'),
})
p r
r.prompt