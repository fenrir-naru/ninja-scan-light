#!/usr/bin/ruby
#coding cp932

class R_Bootstrap
  class <<self
    def download(url = 'https://downloads.sourceforge.net/project/rportable/R-Portable/3.5.3/R-Portable_3.5.3.paf.exe?use_mirror=jaist')
      require 'open-uri'
      len = nil
      $stderr.print "Downloading latest R installer "
      open(url, 'rb',
          :content_length_proc => proc{|content_length| len = content_length},
          :progress_proc => proc{
            step_bytes = nil
            steps = 0
            proc{|transferred_bytes|
              step_bytes ||= len ? (len / 100) : 1_000_000
              next_steps = (transferred_bytes / step_bytes).to_i
              (steps...next_steps).each{|i|
                $stderr.print (i % 10 == 9 ? (len ? "#" : "*") : ".")
              }
              steps = next_steps
            }
          }.call){|io|
        $stderr.puts
        
        require 'tempfile'
        f_path = Tempfile::open(['R', '.pfa.exe']){|io2|
          io2.binmode 
          io2.write(io.read)
          io2.path
        }
        FileUtils::chmod(0755, f_path)
        cmd = "#{f_path} /destination='#{File::dirname(File::absolute_path(__FILE__))}'"
        $stderr.puts "Running installer (#{cmd}) ..."
        system(cmd)
      }
    end
  end
end

if $0 == __FILE__ then
  R_Bootstrap::download
end