require 'rspec'

$: << File::join(File::dirname(__FILE__), '..', 'build_SWIG')
require 'SylphideMath.so'

require 'matrix'

shared_examples 'Matrix' do
  let(:params){{
    :rc => [8, 8],
    :acceptable_delta => 1E-10,
  }}
  describe "initializer" do
    let(:compare_with){
      params[:rc][0].times.map{params[:rc][1].times.map{gen_elm.call}}
    }
    it 'accepts (i,j)' do
      expect{ mat_type::new(*params[:rc]) }.not_to raise_error
      expect( mat_type::new(*params[:rc]).rows ).to equal(params[:rc][0])
      expect( mat_type::new(*params[:rc]).columns ).to equal(params[:rc][1])
    end
    it 'accepts ([[]])' do
      expect{ mat_type::new(compare_with) }.not_to raise_error
      expect( mat_type::new(compare_with).rows ).to equal(params[:rc][0])
      expect( mat_type::new(compare_with).columns ).to equal(params[:rc][1])
    end
    it 'accepts (Matrix)' do
      expect{ mat_type::new(Matrix::build(*params[:rc]){0}) }.not_to raise_error
      expect( mat_type::new(Matrix::build(*params[:rc]){0}).rows ).to equal(params[:rc][0])
      expect( mat_type::new(Matrix::build(*params[:rc]){0}).columns ).to equal(params[:rc][1])
    end
    it 'is invoked with I, identity, unit' do
      [:I, :identity, :unit].each{|f|
        expect{ mat_type::send(f, params[:rc][0]) }.not_to raise_error
        expect( mat_type::send(f, params[:rc][0]).rows ).to equal(params[:rc][0])
        expect( mat_type::send(f, params[:rc][0]).columns ).to equal(params[:rc][1])
      }
    end
    it 'is invoked with scalar' do
      expect{ mat_type::scalar(params[:rc][0], 1) }.not_to raise_error
      expect( mat_type::scalar(params[:rc][0], 1).rows ).to equal(params[:rc][0])
      expect( mat_type::scalar(params[:rc][0], 1).columns ).to equal(params[:rc][1])
    end
    it 'sets its elements with [], [[]], Matrix' do
      expect( mat_type::new(params[:rc][0], params[:rc][1], compare_with.flatten).to_a ).to eq(compare_with)
      expect{ mat_type::new(params[:rc][0], params[:rc][1], compare_with.flatten[0..-2]) }.to raise_error(RuntimeError)
      expect( mat_type::new(params[:rc][0], params[:rc][1], compare_with.flatten + [gen_elm.call]).to_a ).to eq(compare_with)
      expect{ mat_type::new(params[:rc][0], params[:rc][1], compare_with.flatten[0..-2] + [nil]) }.to raise_error(RuntimeError)
  
      expect( mat_type::new(params[:rc][0], params[:rc][1], compare_with).to_a ).to eq(compare_with)
      expect{ mat_type::new(params[:rc][0], params[:rc][1], compare_with[0..-2]) }.to raise_error(RuntimeError)
      expect( mat_type::new(params[:rc][0], params[:rc][1], compare_with + [params[:rc][1].times.map{gen_elm.call}]).to_a ).to eq(compare_with)
      expect( mat_type::new(compare_with).to_a ).to eq(compare_with)
      
      expect( mat_type::new(Matrix[*compare_with]).to_a ).to eq(compare_with)
    end
    it 'sets its elements with {}' do
      expect( mat_type::new(*params[:rc]){|i, j| compare_with[i][j]}.to_a ).to eq(compare_with)
      expect{ mat_type::new(*params[:rc]){nil}.to_a }.to raise_error(RuntimeError)
      expect{ mat_type::new(compare_with){|i, j| compare_with[i][j]} }.to raise_error(ArgumentError)
      expect{ mat_type::new(Matrix[*compare_with]){|i, j| compare_with[i][j]} }.to raise_error(ArgumentError)
    end
  end
  
  describe 'property' do
    let(:mat){{
      :square => proc{
        k = 0
        mat_type::new(params[:rc][0], params[:rc][0]){|i, j| k += 1}
      }.call,
      :not_square => proc{
        k = 0
        mat_type::new(params[:rc][0], params[:rc][0] * 2){|i, j| k += 1}
      }.call,
      :diagonal => mat_type::new(params[:rc][0], params[:rc][0]){|i, j| i == j ? 1 : 0},
      :symmetric => mat_type::new(params[:rc][0], params[:rc][0]){|i, j| i + j},
    }}
    describe 'is checked with' do
      it 'square?' do
        expect(mat[:square].square?)    .to eq(true)
        expect(mat[:not_square].square?).to eq(false)
        expect(mat[:diagonal].square?)  .to eq(true)
        expect(mat[:symmetric].square?) .to eq(true)
      end
      it 'diagonal?' do
        expect(mat[:square].diagonal?)      .to eq(false)
        expect(mat[:not_square].diagonal?)  .to eq(false)
        expect(mat[:diagonal].diagonal?)    .to eq(true)
        expect(mat[:symmetric].diagonal?)   .to eq(false)
      end
      it 'symmetric?' do
        expect(mat[:square].symmetric?)     .to eq(false)
        expect(mat[:not_square].symmetric?) .to eq(false)
        expect(mat[:diagonal].symmetric?)   .to eq(true)
        expect(mat[:symmetric].symmetric?)  .to eq(true)
      end
      it 'different_size?' do
        mat.keys.combination(2).each{|mat1, mat2|
          expect(mat[mat1].different_size?(mat[mat2])).to eq([mat1, mat2].include?(:not_square))
        }
      end
    end
    describe 'is calculated with' do
      it 'trace, tr' do
        [:trace, :tr].each{|f|
          expect(mat[:square].send(f)).to eq(Matrix[*mat[:square].to_a].trace)
          expect{mat[:not_square].send(f)}.to raise_error(RuntimeError)
        }
      end
      it 'sum' do
        expect(mat[:square].sum).to eq(Matrix[*mat[:square].to_a].sum)
        expect(mat[:not_square].sum).to eq(Matrix[*mat[:not_square].to_a].sum)
      end
      it 'determinat, det' do
        [:determinant, :det].each{|f|
          #expect(mat[:square].send(f)).to eq(Matrix[*mat[:square].to_a].det)
          expect{mat[:not_square].send(f)}.to raise_error(RuntimeError)
        }
      end
    end
  end
  
  describe 'element' do
    let(:compare_with){
      2.times.collect{params[:rc][0].times.map{params[:rc][1].times.map{gen_elm.call}}}
    }
    let(:mat){
      compare_with.collect{|item| mat_type::new(item)}
    }
    it 'is accessible with []' do
      params[:rc][0].times{|i|
        params[:rc][1].times{|j|
          expect(mat[0][i, j]).to eq(compare_with[0][i][j])
        }
      }
    end
    it 'is changeable with []=' do
      params[:rc][0].times{|i|
        params[:rc][1].times{|j|
          mat[0][i, j] = compare_with[1][i][j]
          expect(mat[0][i, j]).to eq(compare_with[1][i][j])
        }
      }
    end
  end
  
  describe 'elements' do
    let(:compare_with){
      2.times.collect{params[:rc][0].times.map{params[:rc][1].times.map{gen_elm.call}}}
    }
    let(:mat){
      compare_with.collect{|item| mat_type::new(item)}
    }
    it 'is cloneable with copy' do
      mat_orig, mat_clone = [mat[0], mat[0].copy]
      expect(mat_clone).not_to equal(mat_orig)
      expect(mat_clone.to_a).to eq(mat_orig.to_a)
      mat_clone[0, 0] *= 2
      expect(mat_clone.to_a).not_to eq(mat_orig.to_a)
    end
    it 'is replaceable by using replace! with [], [[]], Matrix or {}' do
      [
        compare_with[1].flatten,    # []
        compare_with[1],            # [[]]
        mat[1],                     # Matrix
        Matrix[*compare_with[1]],   # Matrix(builtin)
        proc{|i, j| mat[1][i, j]},  # {}
      ].each{|arg|
        mat_orig = mat[0].copy
        mat_replaced = arg.kind_of?(Proc) ? mat_orig.send(:replace!, &arg) : mat_orig.send(:replace!, arg)
        expect(mat_replaced).to equal(mat_orig)
        expect(mat_replaced).not_to equal(mat[1])
        expect(mat_replaced.to_a).to eq(mat[1].to_a)
      }
    end
    it 'is swappable with swap_rows! or swap_cloumns!' do
      mat_builtin = Matrix[*compare_with[0]]
      [:swap_rows!, :swap_columns!].each.with_index{|func, i|
        params[:rc][i].times.to_a.combination(2).to_a{|a, b|
          mat[0].send(func, a, b)
          mat_builtin.send(func, a, b)
          expect(mat[0].to_a).to eq(mat_builtin.to_a)
        }
      }
    end
    it 'is checked in their equality by using ==' do
      expect(mat[0] == mat[0]).to be(true)
      expect(mat[0] == mat[0].copy).to be(true)
      expect(mat[0].to_a == compare_with[0]).to be(true)
      expect{mat[0] == compare_with[0]}.to raise_error(ArgumentError)
      expect(mat[0].to_a == Matrix[*compare_with[0]].to_a).to be(true)
      expect{mat[0] == Matrix[*compare_with[0]]}.to raise_error(ArgumentError)
    end
  end
  
  describe 'view' do
    let(:compare_with){params[:rc][0].times.map{params[:rc][1].times.map{gen_elm.call}}}
    let(:mat){mat_type::new(compare_with)}
    it 'supports transposed with transpose, t' do
      expect(mat.transpose.to_a).to eq(Matrix[*compare_with].transpose.to_a)
      expect(mat.t.to_a).to eq(Matrix[*compare_with].t.to_a)
    end
    it 'supports conjugated with conjugate' do
      expect(mat.conjugate.to_a).to eq(Matrix[*compare_with].conjugate.to_a) if mat.respond_to?(:conjugate)
      expect(mat.conjugate.to_a).to eq(Matrix[*compare_with].conj.to_a)
    end
    it 'supports adjointed with adjoint' do
      expect(mat.adjoint.to_a).to eq(Matrix[*compare_with].conjugate.transpose.to_a) if mat.respond_to?(:adjoint)
      expect(mat.adjoint.to_a).to eq(Matrix[*compare_with].conj.t.to_a)
    end
    it 'supports submatrix with partial' do
      expect(mat.partial(params[:rc][0] - 1, params[:rc][1] - 1, 1, 1).to_a) \
          .to eq(Matrix[*compare_with[1..-1].collect{|values| values[1..-1]}].to_a)
    end
    it 'supports row_vector with row_vector' do
      params[:rc][0].times{|i|
        expect(mat.row_vector(i).to_a[0]) \
            .to eq(Matrix[*compare_with].row_vectors[i].to_a)
      }
    end
    it 'supports column_vector with column_vector' do
      params[:rc][1].times{|j|
        expect(mat.column_vector(j).to_a.flatten) \
            .to eq(Matrix[*compare_with].column_vectors[j].to_a)
      }
    end
    it 'generates circular matrix with circular' do
      params[:rc][0].times{|i|
        params[:rc][1].times{|j|
          expect(mat.circular(*([i, j] + params[:rc])).to_a) \
              .to eq(Matrix[*compare_with.collect{|v| v.rotate(j)}.rotate(i)].to_a)
        }
      }
    end
  end
  
  describe 'iterator' do
    let(:compare_with){params[:rc][0].times.map{params[:rc][1].times.map{gen_elm.call}}}
    let(:mat){mat_type::new(compare_with)}
    let(:opt){{
      nil => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a),
      :all => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a),
      :diagonal => params[:rc].min.times.collect{|i| [i, i]},
      :off_diagonal => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a).reject{|i, j| i == j},
      :lower => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a).select{|i, j| i >= j},
      :upper => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a).select{|i, j| i <= j},
      :strict_lower => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a).select{|i, j| i > j},
      :strict_upper => params[:rc][0].times.to_a.product(params[:rc][1].times.to_a).select{|i, j| i < j},
    }}
    it 'supports each, each_with_index' do
      [:each, :each_with_index].each{|func|
        opt.each{|k, indices|
          candidates = (func.to_s =~ /with_index$/) \
              ? indices.collect{|i, j| [compare_with[i][j], i, j]} \
              : indices.collect{|i, j| [compare_with[i][j]]}
          mat.send(*[func, k].compact){|*v|
            i = candidates.find_index(v)
            expect(i).not_to be(nil)
            candidates.delete_at(i)
          }
          expect(candidates.empty?).to be(true)
        }
      }
    end
    it 'supports map, collect, map_with_index, collect_with_index' do
      [:map, :collect, :map_with_index, :collect_with_index].each{|func|
        opt.each{|k, indices|
          candidates = (func.to_s =~ /with_index$/) \
              ? indices.collect{|i, j| [compare_with[i][j], i, j]} \
              : indices.collect{|i, j| [compare_with[i][j]]}
          mat2 = mat.send(*[func, k].compact){|*v|
            i = candidates.find_index(v)
            expect(i).not_to be(nil)
            candidates.delete_at(i)
            v[0] * 2
          }
          expect(candidates.empty?).to be(true)
          expect(mat.to_a).to eq(compare_with)
          expect(mat2.to_a).to eq(compare_with.collect.with_index{|values, i|
                values.collect.with_index{|v, j|
                  indices.include?([i, j]) ? (v * 2) : v
                }
              })
        }
      }
    end
    it 'supports map!, collect!, map_with_index!, collect_with_index!' do
      [:map!, :collect!].each{|func|
        opt.each{|k, indices|
          candidates = (func.to_s =~ /with_index$/) \
              ? indices.collect{|i, j| [compare_with[i][j], i, j]} \
              : indices.collect{|i, j| [compare_with[i][j]]}
          mat = mat_type::new(compare_with)
          mat2 = mat.send(*[func, k].compact){|*v|
            i = candidates.find_index(v)
            expect(i).not_to be(nil)
            candidates.delete_at(i)
            v[0] * 2
          }
          expect(candidates.empty?).to be(true)
          expect(mat2.to_a).to eq(mat.to_a)
          expect(mat2.to_a).to eq(compare_with.collect.with_index{|values, i|
                values.collect.with_index{|v, j|
                  indices.include?([i, j]) ? (v * 2) : v
                }
              })
        }
      }
    end
  end
  
  describe 'operators' do
    let(:compare_with){ # [N*N, N*N, N*(N+1), N*(N+1)]
      2.times.collect{params[:rc][0].times.map{params[:rc][0].times.map{gen_elm.call}}} \
          + 2.times.collect{params[:rc][0].times.map{(params[:rc][0] + 1).times.map{gen_elm.call}}}
    }
    let(:mat){
      compare_with.collect{|item| mat_type::new(item)}
    }
    it 'have unary -' do
      [0, 2].each{|i|
        expect((-mat[i]).to_a).to eq((Matrix[*compare_with[i]] * -1).to_a)
      }
    end
    it 'have +(scalar)' do
      expect((mat[0] + 1).to_a).to eq((Matrix[*compare_with[0]] + Matrix::unit(params[:rc][0])).to_a)
      expect{mat[2] + 1}.to raise_error(RuntimeError)
    end
    it 'have +(mat)' do
      [[0, 1], [2, 3]].each{|i, j|
        expect((mat[i] + mat[j]).to_a).to eq((Matrix[*compare_with[i]] + Matrix[*compare_with[j]]).to_a)
      }
    end
    it 'have -(scalar)' do
      expect((mat[0] - 1).to_a).to eq((Matrix[*compare_with[0]] - Matrix::unit(params[:rc][0])).to_a)
      expect{mat[2] - 1}.to raise_error(RuntimeError)
    end
    it 'have -(mat)' do
      [[0, 1], [2, 3]].each{|i, j|
        expect((mat[i] - mat[j]).to_a).to eq((Matrix[*compare_with[i]] - Matrix[*compare_with[j]]).to_a)
      }
    end
    it 'have *(scalar)' do
      [0, 2].each{|i|
        expect((mat[i] * 2).to_a).to eq((Matrix[*compare_with[i]] * 2).to_a)
      }
    end
    it 'have *(mat)' do
      expect((mat[0] * mat[1]).to_a).to eq((Matrix[*compare_with[0]] * Matrix[*compare_with[1]]).to_a)
      expect{mat[2] * mat[3]}.to raise_error(RuntimeError)
      expect((mat[2] * mat[3].t).to_a).to eq((Matrix[*compare_with[2]] * Matrix[*compare_with[3]].t).to_a)
    end
    it 'have /(scalar)' do
      expect((mat[0] / 2).to_a).to eq((Matrix[*compare_with[0]] / 2).to_a)
      expect((mat[2] / 2).to_a).to eq((Matrix[*compare_with[2]] / 2).to_a)
    end
    let(:inversible){
      # L * U
      mat_type::new(compare_with[0]).map_with_index!{|v, i, j| (i >= j) ? (i == j ? 1 : v) : 0} \
        * mat_type::new(compare_with[1]).map_with_index!{|v, i, j| (i <= j) ? (i == j ? 1 : v) : 0}
    }
    it 'have inverse, inv' do
      [:inverse, :inv].each{|func|
        (Matrix[*(inversible.send(func).to_a)] - Matrix[*(inversible.to_a)].send(func)).each{|v| 
          expect(v.abs).to be < params[:acceptable_delta]
        }
        expect{mat[2].send(func)}.to raise_error(RuntimeError)
      }
    end
    it 'have /(mat)' do
      (Matrix[*((mat[0] / mat[1]).to_a)] - (Matrix[*compare_with[0]] / Matrix[*compare_with[1]])).each{|v|
        expect(v.abs).to be < params[:acceptable_delta]
      }
      expect{mat[2] / mat[3]}.to raise_error(RuntimeError)
    end
  end
  
  describe 'decomposition' do
    let(:compare_with){ params[:rc][0].times.map{params[:rc][0].times.map{gen_elm.call} } }
    let(:mat){{
      :L => mat_type::new(compare_with).map_with_index!{|v, i, j| (i >= j) ? (i == j ? 1 : v) : 0},
      :U => mat_type::new(compare_with).map_with_index!{|v, i, j| (i <= j) ? (i == j ? 1 : v) : 0},
    }}
    it 'supports LU' do
      src = mat_type::new(Matrix[*mat[:L].to_a] * Matrix[*mat[:U].to_a])
      [:lup, :lup_decomposition].each{|func|
        mat_l, mat_u, mat_p = src.send(func).collect{|item| Matrix[*item.to_a]}
        expect(mat_l.lower_triangular?).to be(true)
        expect(mat_u.upper_triangular?).to be(true)
        ((mat_l * mat_u * mat_p) - Matrix[*src.to_a]).each{|v|
          expect(v.abs).to be < params[:acceptable_delta]
        }
      }
    end
    it 'supports QR' do
      src = mat_type::new(Matrix[*mat[:L].to_a] * Matrix[*mat[:U].to_a])
      [:qr, :qr_decomposition].each{|func|
        [[0, 0], [2, 0], [0, 2]].each{|i, j|
          src2 = src.partial(src.rows - i, src.columns - j, i, j)
          mat_q, mat_r = src2.send(func).collect{|item| Matrix[*item.to_a]}
          expect(mat_r.upper_triangular?).to be(true)
          expect((mat_q.det - 1).abs).to be < params[:acceptable_delta]
          ((mat_q * mat_r) - Matrix[*src2.to_a]).each{|v|
            expect(v.abs).to be < params[:acceptable_delta]
          }
          ((mat_q * mat_q.t.conj) - Matrix::unit(mat_q.row_size)).each{|v|
            expect(v.abs).to be < params[:acceptable_delta]
          }
        }
      }
    end
  end
end

=begin
TODO
:debug
:to_s
:to_a
=end

describe SylphideMath::MatrixD do
  let(:mat_type){SylphideMath::MatrixD}
  let(:gen_elm){proc{rand}}
  include_examples 'Matrix'
  
  describe 'decomposition' do
    let(:compare_with){ params[:rc][0].times.map{params[:rc][0].times.map{gen_elm.call} } }
    let(:mat){{
      :L => mat_type::new(compare_with).map_with_index!{|v, i, j| (i >= j) ? (i == j ? 1 : v) : 0},
      :U => mat_type::new(compare_with).map_with_index!{|v, i, j| (i <= j) ? (i == j ? 1 : v) : 0},
      :D => mat_type::new(compare_with).map_with_index!{|v, i, j| (i == j) ? v : 0},
      :Q => mat_type::new(params[:rc][0].times.map{(params[:rc][0] * 2).times.map{gen_elm.call}}).qr[0],
    }}
    it 'supports UD' do
      src = mat_type::new((Matrix[*mat[:U].to_a] * Matrix[*mat[:D].to_a] * Matrix[*mat[:U].t.to_a]).to_a)
      src.map_with_index!(:strict_upper){|v, i, j| src[j, i]}
      expect(Matrix[*src.to_a].symmetric?).to be(true)
      [:ud, :ud_decomposition].each{|func|
        mat_u, mat_d = src.send(func).collect{|item| Matrix[*item.to_a]}
        expect(mat_u.upper_triangular?).to be(true)
        expect(mat_d.diagonal?).to be(true)
        ((mat_u * mat_d * mat_u.t) - Matrix[*src.to_a]).each{|v|
          expect(v.abs).to be < params[:acceptable_delta]
        }
      }
    end
    it 'supports eigenvalue' do
      mat_q, mat_u = [:Q, :U].collect{|k| Matrix[*mat[k].map{|v| v}.to_a]}
      expect((mat_q.det - 1).abs).to be < params[:acceptable_delta]
      expect(mat_u.det).to eq(1)
      src = mat_q * mat_u * Matrix[*mat[:D].to_a]
      mat_v, mat_d = mat_type::new(src.to_a).eigen.collect{|item| Matrix[*item.to_a]}
      expect(mat_d.diagonal?).to be(true)
      mat_d.row_size.times{|i|
        (src * mat_v.column_vectors[0] - mat_v.column_vectors[0] * mat_d[0, 0]).each{|v|
          expect(v.abs).to be < params[:acceptable_delta]
        }
      }
    end
  end
end

describe SylphideMath::MatrixComplexD do
  let(:mat_type){SylphideMath::MatrixComplexD}
  let(:gen_elm){proc{Complex(rand, rand)}}
  include_examples 'Matrix'
end
