/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "param/matrix.h"

/** @file
 * @brief Kalman Filterを記述したファイルです。
 * 
 * Kalman　Filterを記述したファイルです。
 * 現在は2種類の離散Kalman Filter、すなわち標準的なKalman Filterと
 * UD分解Klamn Filterをサポートしています。
 * 内部的に行列の計算を行っているため、行列ライブラリ( Matrix )を必要とします。
 * 
 * @see Matrix 行列ライブラリ 
 */

/**
 * @brief 標準的なKalman Filter
 * 
 * 標準的なKalman Filterを定義しています。
 * 使用するためにはシステムの誤差共分散行列P、入力の誤差共分散行列Qを初期化時に指定する必要があります。
 * 初期化後はinit()を呼び出して初期化を完了させてください。
 * 時間更新(Time Update)ではシステム方程式に関連する行列@f$ A @f$, @f$ B @f$ 
 * またはそれをEuler積分した@f$ \Phi @f$または@f$ \Gamma @f$が必要です。
 * @f$ A @f$, @f$ B @f$の定義は状態量@f$ x @f$、並びに入力@f$ u @f$に対して
 * @f[
 *    \frac{d}{dt} x = A x + B u
 * @f]
 * で定義され
 * @f[
 *    \Phi = \int_{t} A, \Gamma = \int_{t} B
 * @f]
 * です。
 * 観測更新(Measurement Update)では観測方程式に関連する行列@f$ H @f$、
 * 観測誤差共分散行列@f$ R @f$、並びに観測量@f$ z @f$が必要とされます。
 * これらの定義は
 * @f[
 *    z = H x + v
 * @f] 
 * です。
 * 
 * なお、テンプレートFloatTは演算精度の型を指定します。
 * よほどのことがない限り、doubleを推奨します。
 * 
 * @param FloatT 演算精度
 */
template <class FloatT>
class KalmanFilter{
  protected:
    Matrix<FloatT> m_P; ///< カルマンフィルタのP行列(システム誤差共分散行列)
    Matrix<FloatT> m_Q; ///< カルマンフィルタのQ行列(入力誤差共分散行列)
    
  public:
    /**
     * KalmanFilterのコンストラクタ。
     * 誤差共分散行列@f$ P @f$, @f$ Q @f$を指定する必要があります。
     * 
     * @param P @f$ P @f$行列
     * @param Q @f$ Q @f$行列
     */
    KalmanFilter(const Matrix<FloatT> &P,
                 const Matrix<FloatT> &Q) : m_P(P), m_Q(Q) {
    }
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    KalmanFilter(const KalmanFilter &orig, const bool &deepcopy = false) :
      m_P(deepcopy ? orig.m_P.copy() : orig.m_P), 
      m_Q(deepcopy ? orig.m_Q.copy() : orig.m_Q){
      //std::cerr << "KF" << std::endl;
      
    }
    
    /**
     * KalmanFilterのデストラクタ。
     * 
     */
    virtual ~KalmanFilter(){}
    
    /**
     * 推定フィルターを時間更新します。
     * 離散系バージョン
     * @f[
     *   x_{k+1} = \Phi_{k+1,K} * x_{k} + \Gamma_{k} * u_{k}
     * @f]
     * 
     * @param Phi @f$ \Phi @f$行列
     * @param Gamma @f$ \Gamma @f$行列
     */
    virtual void predict(const Matrix<FloatT> &Phi, const Matrix<FloatT> &Gamma){
    
#if DEBUG > 2
      std::cerr << "Phi:" << Phi << std::endl;
      std::cerr << "Gamma:" << Gamma << std::endl;
#endif

      (m_P = Phi * m_P * Phi.transpose()) += (Gamma * m_Q * Gamma.transpose());
    }
    
    /**
     * 推定フィルターをEuler法によって時間更新します。
     * 連続系バージョン。
     * @f{gather*}
     *   \frac{d}{dt} x = A x + B u \\
     *   \Phi_{k+1,k} = I + A \Delta t \\
     *   \Gamma_{k} = B \Delta t
     * @f}
     * 
     * @param A @f$ A @f$行列
     * @param B @f$ B @f$行列
     * @param delta 時間間隔 
     */
    virtual void predict(const Matrix<FloatT> &A, const Matrix<FloatT> &B, const FloatT &delta){
      
      //φ
      Matrix<FloatT> Phi = A * delta;
      for(unsigned i = 0; i < Phi.rows(); i++) Phi(i, i) += 1;
    
      //Γ
      Matrix<FloatT> Gamma = B * delta;
      
      predict(Phi, Gamma);
    }
    
    /**
     * フィルターを観測更新(修正)し、その際のカルマンゲインを求めます。
     * 
     * @param H @f$ H @f$行列(観測行列)
     * @param R 観測値の誤差共分散行列@f$ R @f$
     * @return (Matrix<FloatT>) カルマンゲイン@f$ K @f$
     */
    virtual Matrix<FloatT> correct(const Matrix<FloatT> &H, const Matrix<FloatT> &R){

      // カルマンゲインの計算
      Matrix<FloatT> K(m_P * H.transpose() * ((H * m_P * H.transpose()) += R).inverse());
#if DEBUG > 1
      std::cerr << "K:" << K << std::endl;
#endif

      // P 更新
      m_P = (Matrix<FloatT>::getI(K.rows()) - K * H) * m_P;
#if DEBUG
      std::cerr << "P:" << m_P << std::endl;
#endif
      
      return K;
    }
    
    /**
     * 誤差共分散行列@f$ P @f$を返します。
     * 
     * @return (const Matrix<FloatT> &) 現在の@f$ P @f$行列
     */
    virtual const Matrix<FloatT> &getP() const {return m_P;}

    /**
     * 誤差共分散行列@f$ P @f$を設定します。
     *
     * @param P 新しい@f$ P @f$行列
     */
    virtual void setP(const Matrix<FloatT> &P){m_P = P;}
    
    /**
     * 誤差共分散行列@f$ Q @f$を返します。
     * 
     * @return (Matrix<FloatT>) 現在の@f$ Q @f$行列
     */
    virtual const Matrix<FloatT> &getQ() const {return m_Q;}

    /**
     * 誤差共分散行列@f$ Q @f$を設定します。
     *
     * @param Q 新しい@f$ Q @f$行列
     */
    virtual void setQ(const Matrix<FloatT> &Q){m_Q = Q;}
};

/**
 * @brief Information Filter
 * 
 * Information Filter(Kalman Filterでシステム共分散行列が逆行列になる)を定義しています。
 * 使用する上でその使い方は標準的なKalman Filterとなんら変わりません。
 * 
 * @param FloatT 演算精度
 * @see KalmanFilter
 */
template <class FloatT>
class InformationFilter : public KalmanFilter<FloatT>{
  protected:
    Matrix<FloatT> m_I;
    bool need_update_P;
    
    /**
     * 誤差共分散行列@f$ P @f$を更新します。
     * @f$ P @f$の逆行列@f$ I @f$から@f$ P @f$を復元しています。
     * これを行うことによってフィルター内部の整合性が保たれます。
     * 
     */
    void updateP(){
      if(!need_update_P){return;}
      //P更新
      KalmanFilter<FloatT>::m_P = m_I.inverse();
      need_update_P = false;
#if DEBUG
      std::cerr << "P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
    }
    
  public:
    /**
     * 誤差共分散行列@f$ P @f$を返します。
     * 
     * @return (Matrix<FloatT>) 現在の@f$ P @f$行列
     */
    const Matrix<FloatT> &getP() const {
      const_cast<InformationFilter *>(this)->updateP();
      return KalmanFilter<FloatT>::m_P;
    }

    /**
     * 誤差共分散行列@f$ P @f$を設定します。
     *
     * @param P 新しい@f$ P @f$行列
     */
    void setP(const Matrix<FloatT> &P){
      m_I = P.inverse();
      need_update_P = true;
    }
  
    /**
     * UD分解KalmanFilterのコンストラクタ。
     * 誤差共分散行列@f$ P @f$, @f$ Q @f$を指定する必要があります。
     * 
     * @param P @f$ P @f$行列
     * @param Q @f$ Q @f$行列
     */
    InformationFilter(
        const Matrix<FloatT> &P, const Matrix<FloatT> &Q)
          : KalmanFilter<FloatT>(P, Q), m_I(P.inverse()), need_update_P(false){
    }
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    InformationFilter(
        const InformationFilter &orig, const bool &deepcopy = false)
          : KalmanFilter<FloatT>(orig, deepcopy),
            m_I(deepcopy ? orig.m_I.copy() : orig.m_I),
            need_update_P(orig.need_update_P){
      //std::cerr << "KFUD" << std::endl;
    }
    
    /**
     * デストラクタ
     * 
     */
    ~InformationFilter(){}
    
    using KalmanFilter<FloatT>::predict;

    /**
     * 推定フィルターを時間更新します。
     * 離散系バージョン
     * @f[
     *   x_{k+1} = \Phi_{k+1,K} * x_{k} + \Gamma_{k} * u_{k}
     * @f]
     * 
     * @param Phi @f$ \Phi @f$行列
     * @param Gamma @f$ \Gamma @f$行列
     */
    void predict(const Matrix<FloatT> &Phi, const Matrix<FloatT> &Gamma){
     
#if DEBUG     
      KalmanFilter<FloatT>::predict(Phi, Gamma);
      std::cerr << "predict_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif

      Matrix<FloatT> inv_additive_term(
          (Gamma * KalmanFilter<FloatT>::m_Q * Gamma.transpose()).inverse());
      m_I = inv_additive_term
          - inv_additive_term * Phi 
            * (m_I + Phi.transpose() * inv_additive_term * Phi).inverse()
            * Phi.transpose() * inv_additive_term;

      //行列Pの更新
      need_update_P = true;

#if DEBUG
      std::cerr << "predict_IF_P:" << getP() << std::endl;
#endif
    }
    
    /**
     * フィルターを観測更新(修正)し、その際のカルマンゲインを求めます。
     * 内部的にUD分解を利用しています。
     * 
     * @param H @f$ H @f$行列(観測行列)
     * @param R 観測値の誤差共分散行列@f$ R @f$
     * @return (Matrix<FloatT>) カルマンゲイン@f$ K @f$
     */
    Matrix<FloatT> correct(const Matrix<FloatT> &H, const Matrix<FloatT> &R){
#if DEBUG
      std::cerr << "correct_KF_K:" << KalmanFilter<FloatT>::correct(H, R) << std::endl;
      std::cerr << "correct_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
      
      Matrix<FloatT> R_inv(R.inverse());
      Matrix<FloatT> H_trans(H.transpose());
      
      m_I += H_trans * R_inv * H;
      
      // カルマンゲイン
      Matrix<FloatT> K(m_I.inverse() * H_trans * R_inv);
      
      //行列Pの更新
      need_update_P = true;

#if DEBUG
      std::cerr << "correct_IF_K:" << K << std::endl;
      std::cerr << "correct_IF_P:" << getP() << std::endl;
#endif

      return K;
    }
    
    /**
     * 誤差共分散行列@f$ P @f$の逆行列@f$ I @f$を返します。
     * 
     * @return (const Matrix<FloatT> &) 行列@f$ I @f$
     */
    const Matrix<FloatT> &getI(){return m_I;}
};

/**
 * @brief UD分解Kalman Filter
 * 
 * UD分解Kalman Filterを定義しています。
 * 演算精度をあげるために内部的にUD分解を利用していることが標準的なKalman Filterとの違いで、
 * 使用する上でその使い方は標準的なKalman Filterとなんら変わりません。
 * 
 * @param FloatT 演算精度
 * @see KalmanFilter
 */
template <class FloatT>
class KalmanFilterUD : public KalmanFilter<FloatT>{
  protected:
    Matrix<FloatT> m_U, m_D;
    bool need_update_P;
    
    /**
     * 誤差共分散行列@f$ P @f$を更新します。
     * @f$ P @f$をUD分解した行列から@f$ P @f$を復元しています。
     * これを行うことによってフィルター内部の整合性が保たれます。
     * 
     */
    void updateP(){
      if(!need_update_P){return;}
      //P更新
      KalmanFilter<FloatT>::m_P = m_U * m_D * m_U.transpose();
      need_update_P = false;
#if DEBUG
      std::cerr << "P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
    }
    
  public:
    /**
     * 誤差共分散行列@f$ P @f$を返します。
     * 
     * @return (Matrix<FloatT>) 現在の@f$ P @f$行列
     */
    const Matrix<FloatT> &getP(){
      const_cast<KalmanFilterUD *>(this)->updateP();
        return KalmanFilter<FloatT>::m_P;
    }

    /**
     * 誤差共分散行列@f$ P @f$を設定します。
     * 内部的には誤差共分散行列@f$ P @f$のUD分解を行い、後の計算に備えます。
     *
     * @param P 新しい@f$ P @f$行列
     */
    void setP(const Matrix<FloatT> &P){
      m_U = Matrix<FloatT>(P.rows(), P.columns());
      m_D = Matrix<FloatT>(P.rows(), P.columns());

      // UD分解
      Matrix<FloatT> UD(P.decomposeUD(false));

      for(int i = 0; i < m_U.rows(); i++){
        m_D(i, i) = UD(i, i + m_U.columns());
        for(int j = 0; j < m_U.columns(); j++){
          m_U(i, j) = UD(i, j);
        }
      }
#if DEBUG
      std::cerr << "U:" << m_U << std::endl;
      std::cerr << "D:" << m_D << std::endl;
#endif
    }
  
    /**
     * UD分解KalmanFilterのコンストラクタ。
     * 誤差共分散行列@f$ P @f$, @f$ Q @f$を指定する必要があります。
     * 
     * @param P @f$ P @f$行列
     * @param Q @f$ Q @f$行列
     */
    KalmanFilterUD(const Matrix<FloatT> &P,
                   const Matrix<FloatT> &Q)
        : KalmanFilter<FloatT>(P, Q), m_U(), m_D(), need_update_P(false){
      setP(P);
    }
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    KalmanFilterUD(const KalmanFilterUD &orig, const bool &deepcopy = false) :
      KalmanFilter<FloatT>(orig, deepcopy),
      m_U(deepcopy ? orig.m_U.copy() : orig.m_U), 
      m_D(deepcopy ? orig.m_D.copy() : orig.m_D),
      need_update_P(orig.need_update_P){
      //std::cerr << "KFUD" << std::endl;
    }
    
    /**
     * KalmanFilterのデストラクタ。
     * 
     */
    ~KalmanFilterUD(){}
    
    using KalmanFilter<FloatT>::predict;

    /**
     * 推定フィルターを時間更新します。
     * 内部的にUD分解を活用しています。
     * 離散系バージョン
     * @f[
     *   x_{k+1} = \Phi_{k+1,K} * x_{k} + \Gamma_{k} * u_{k}
     * @f]
     * 
     * @param Phi @f$ \Phi @f$行列
     * @param Gamma @f$ \Gamma @f$行列
     */
    void predict(const Matrix<FloatT> &Phi, const Matrix<FloatT> &Gamma){
     
#if DEBUG     
      KalmanFilter<FloatT>::predict(Phi, Gamma);
      std::cerr << "predict_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif

      // 行列FU
      Matrix<FloatT> FU(Phi * m_U);
      
      // 行列W
      Matrix<FloatT> W(Phi.rows(), KalmanFilter<FloatT>::m_P.columns() + Gamma.columns());
      W.pivotMerge(0, 0, FU);
      W.pivotMerge(0, Phi.rows(), Gamma);
      
      // 行列Q
      Matrix<FloatT> Q(W.columns(), W.columns());
      for(int i = 0; i < m_D.rows(); i++){
        Q(i, i) = m_D(i, i);
      }
      for(int i = m_D.rows(); i < Q.rows(); i++){
        Q(i, i) = KalmanFilter<FloatT>::m_Q(i - m_D.rows(), i - m_D.rows());
      }

#if DEBUG
      std::cerr << "W:" << W << std::endl;
      std::cerr << "Q:" << Q << std::endl;
#endif
      
      for(int j = W.rows() - 1; j > 0; j--){
        typename Matrix<FloatT>::partial_t V(W.rowVector(j));
        
        Matrix<FloatT> Z(1, Q.columns()); // = V * Q、高速化のため展開して書く
        for(int i = 0; i < Z.columns(); i++){
          Z(0, i) = V(0, i) * Q(i, i); 
        }
        
        m_D(j, j) = (Z * V.transpose())(0, 0);
        for(int i = 0; i < j; i++){
          m_U(i, j) = (W.rowVector(i) * Z.transpose())(0, 0) / m_D(j, j);
          W.rowVector(i) -= m_U(i, j) * V;
        }
      }
      
      // m_D(0, 0) = (W.rowVector(0) * Q * W.rowVector(0).transpose())(0, 0);→高速化
      m_D(0, 0) = 0;
      for(int j = 0; j < W.columns(); j++){
        m_D(0, 0) += W(0, j) * W(0, j) * Q(j, j);
      }

      //行列Pの更新
      need_update_P = true;

#if DEBUG
      std::cerr << "predict_UDKF_U:" << m_U << std::endl;
      std::cerr << "predict_UDKF_D:" << m_D << std::endl;
      std::cerr << "predict_UDKF_P:" << getP() << std::endl;
#endif
    }
    
    /**
     * フィルターを観測更新(修正)し、その際のカルマンゲインを求めます。
     * 内部的にUD分解を利用しています。
     * 
     * @param H @f$ H @f$行列(観測行列)
     * @param R 観測値の誤差共分散行列@f$ R @f$
     * @return (Matrix<FloatT>) カルマンゲイン@f$ K @f$
     */
    Matrix<FloatT> correct(const Matrix<FloatT> &H, const Matrix<FloatT> &R){
#if DEBUG
      std::cerr << "correct_KF_K:" << KalmanFilter<FloatT>::correct(H, R) << std::endl;
      std::cerr << "correct_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
      
      // カルマンゲイン
      Matrix<FloatT> K(KalmanFilter<FloatT>::m_P.rows(), R.rows());
      
      for(int k = 0; k < R.rows(); k++){
        Matrix<FloatT> f(m_U.columns(), H.rows());
        Matrix<FloatT> g(m_D.rows(), f.columns());
        
        // fの生成
        for(int i = 0; i < f.rows(); i++){
          for(int j = 0; j <= i; j++){
            f(i, 0) += H(k, j) * m_U(j, i);
          }
        }
        
        // gの生成
        for(int i = 0; i < g.rows(); i++){
          g(i, 0) = m_D(i, i) * f(i, 0);
        }
        
        FloatT r(R(k, k));
        FloatT alpha = r + f(0, 0) * g(0, 0);
        K(0, k) = g(0, 0);
        m_D(0, 0) *= (r / alpha);
        
        for(int j = 1; j < f.rows(); j++){
          FloatT _alpha(alpha + f(j, 0) * g(j, 0));
          m_D(j, j) *= (alpha / _alpha);
          FloatT lambda(f(j, 0) / alpha);
          Matrix<FloatT> _u(m_U.columnVector(j).copy());
          m_U.columnVector(j) -= lambda * K.columnVector(k);
          K.columnVector(k) += g(j, 0) * _u;
          alpha = _alpha;
        }
        K.columnVector(k) /= alpha;
      }
      
      //行列Pの更新
      need_update_P = true;

#if DEBUG
      std::cerr << "correct_UDKF_K:" << K << std::endl;
      std::cerr << "correct_UDKF_P:" << getP() << std::endl;
#endif

      return K;
    }
    
    /**
     * 誤差共分散行列@f$ P @f$をUD分解したうちの行列@f$ U @f$を返します。
     * 
     * @return (Matrix<FloatT>) 行列@f$ U @f$
     */
    const Matrix<FloatT> &getU() const {return m_U;}
    
    /**
     * 誤差共分散行列@f$ P @f$をUD分解したうちの行列@f$ D @f$を返します。
     * 
     * @return (Matrix<FloatT>) 行列@f$ D @f$
     */
    const Matrix<FloatT> &getD() const {return m_D;}
};

/**
 * @brief UnscentedKalman Filter
 * 
 * Unscented Kalman Filterを定義しています。
 * 
 * @param FloatT 演算精度
 * @see KalmanFilter
 */
template <class FloatT>
class UnscentedKalmanFilter : public KalmanFilter<FloatT>{
  protected:
    FloatT m_alpha, m_beta, m_kappa;
    
    bool need_recalc_coef;
    FloatT gamma, lambda;
    unsigned n_a;
    FloatT weightM_0, weightC_0, weight_i;
    Matrix<FloatT> m_sqrtQ;
    
    /**
     * 共分散行列のsqrt(成分が常に実数となる)を求めます。
     * 
     * @param cov 共分散行列
     * @return (Matrix<FloatT>) sqrtされた行列
     */
    Matrix<FloatT> get_sqrt_cov(const Matrix<FloatT> &cov){
      Matrix<FloatT> sqrt_cov(cov.rows(), cov.columns());
      
      if(cov.isDiagonal()){   // 対角行列の場合、高速化
        for(int i(0); i < sqrt_cov.rows(); i++){
          sqrt_cov(i, i) = sqrt(cov(i, i));
        }
      }else{  // そうでない場合はまじめに計算する
        Matrix<Complex<FloatT> > sqrt_cov_C(cov.sqrt());
        for(int i(0); i < sqrt_cov_C.rows(); i++){
          for(int j(0); j < sqrt_cov_C.columns(); j++){
            sqrt_cov(i, j) = sqrt_cov_C(i, j).real();
          }
        }
      }
      return sqrt_cov;
    }
    
    /**
     * 係数などフィルター演算に必要な値のキャッシュを更新します。
     * 
     */
    void recalc_coef(){
      if(!need_recalc_coef){return;}
      
      n_a = KalmanFilter<FloatT>::m_P.rows();
      lambda = pow(m_alpha, 2) * (m_kappa + n_a) - n_a;
      FloatT gamma2(lambda + n_a);
      gamma = ::sqrt(gamma2);
      weightM_0 = lambda / gamma2;
      weightC_0 = lambda / gamma2 + (FloatT(1) - pow(m_alpha, 2) + m_beta);
      weight_i = FloatT(1) / (gamma2 * 2);
      
      m_sqrtQ = get_sqrt_cov(KalmanFilter<FloatT>::m_Q);
      
      need_recalc_coef = false;
    }
    
  public:
    /**
     * UnscentedKalmanFilterのコンストラクタ。
     * 誤差共分散行列@f$ P @f$, @f$ Q @f$を指定する必要があります。
     * 
     * @param P @f$ P @f$行列
     * @param Q @f$ Q @f$行列
     */
    UnscentedKalmanFilter(const Matrix<FloatT> &P, const Matrix<FloatT> &Q)
        : KalmanFilter<FloatT>(P, Q),
          m_alpha(1),   // typically 0.001 - 1 (P.239, 以下同じ) 
          m_beta(2),    // the optiomal value for Gaussian distribution
          m_kappa(0),   // 0 or 3 - n_a
          need_recalc_coef(true), m_sqrtQ(){
    }
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    UnscentedKalmanFilter(const UnscentedKalmanFilter &orig, const bool &deepcopy = false)
        : KalmanFilter<FloatT>(orig, deepcopy),
          m_alpha(orig.m_alpha), m_beta(orig.m_beta), m_kappa(orig.m_kappa), 
          need_recalc_coef(true), m_sqrtQ(){
      //std::cerr << "UKF" << std::endl;
    }
    
    /**
     * @f$ \alpha @f$ を取得します
     * 
     * @return (FloatT &)
     */
    FloatT &alpha(){
      need_recalc_coef = true;
      return m_alpha;
    }
    
    /**
     * @f$ \beta @f$ を取得します
     * 
     * @return (FloatT &)
     */
    FloatT &beta(){
      need_recalc_coef = true;
      return m_beta;
    } 
    
    /**
     * @f$ \kappa @f$ を取得します
     * 
     * @return (FloatT &)
     */
    FloatT &kappa(){
      need_recalc_coef = true;
      return m_kappa;
    }
    
    /**
     * 誤差共分散行列@f$ P @f$を設定します。
     *
     * @param P 新しい@f$ P @f$行列
     */
    void setP(const Matrix<FloatT> &P){
      need_recalc_coef = true;
      KalmanFilter<FloatT>::m_P = P;
    }

    /**
     * 誤差共分散行列@f$ Q @f$を設定します。
     *
     * @param Q 新しい@f$ Q @f$行列
     */
    void setQ(const Matrix<FloatT> &Q){
      need_recalc_coef = true;
      KalmanFilter<FloatT>::m_Q = Q;
    }
    
    /**
     * KalmanFilterのデストラクタ。
     * 
     */
    ~UnscentedKalmanFilter(){}
    
  protected:
    template <class StateValues>
    void get_perturbed_states(StateValues &state, StateValues *state_with_perturbation){
      Matrix<Complex<FloatT> > sqrtP(KalmanFilter<FloatT>::m_P.sqrt());
      for(unsigned k(0); k < n_a; k++){
        for(unsigned i(0); i < n_a; i++){
          FloatT perturbation(sqrtP(i, k).real());
          state_with_perturbation[k][i] = state[i] + gamma * perturbation;
          state_with_perturbation[k + n_a][i] = state[i] - gamma * perturbation;
        }
      }
    }
  
  public:
    /**
     * 状態量とフィルターを時間更新します。
     * 
     * @param functor 時間更新関数(2または3引数をとるoperator()が定義されていること)
     * @param state 状態量([]が定義されていること)
     * @param input システムへの入力
     */
    template <class TimeUpdateFunctor, class StateValues, class InputValues>
    void predict(TimeUpdateFunctor &functor, StateValues &state, InputValues &input){
      recalc_coef();
      
      // 偏差分だけずれた状態量(シグマポイント)を計算
      StateValues *state_sigma(new StateValues [n_a * 2]);
      get_perturbed_states(state, state_sigma);
      
      // 次のステップの計算とmeanの計算(状態量の更新)
      StateValues state0_next = functor(state, input);
      for(unsigned i(0); i < n_a; i++){
        state[i] = weightM_0 * state0_next[i];
      }
      for(unsigned k(0); k < n_a; k++){
        state_sigma[k] = functor(state_sigma[k], input, m_sqrtQ);
        state_sigma[k + n_a] = functor(state_sigma[k + n_a], input, -m_sqrtQ);
        for(unsigned i(0); i < n_a; i++){
          state[i] += weight_i * state_sigma[k][i];
          state[i] += weight_i * state_sigma[k + n_a][i];
        }
      }
      
      // covの計算
      {
        Matrix<FloatT> P_vec(n_a, 1);
        
        // i = 0
        for(unsigned i(0); i < n_a; i++){
          P_vec(i, 0) = state0_next[i] - state[i];
        }
        KalmanFilter<FloatT>::m_P = P_vec * P_vec.transpose() * weightC_0;
        
        // i > 0
        for(unsigned k(0); k < n_a * 2; k++){
          for(unsigned i(0); i < n_a; i++){
            P_vec(i, 0) = state_sigma[k][i] - state[i];
          }
          KalmanFilter<FloatT>::m_P += P_vec * P_vec.transpose() * weight_i;
        }
      }
      
      delete [] state_sigma;
    }
    
    /**
     * 状態量とフィルターを観測更新(修正)し、その際のカルマンゲインを求めます。
     * 
     * @param functor 観測方程式(1引数をとるoperator()が定義されていること)
     * @param state 状態量([]が定義されていること)
     * @param z 観測量([]、variablesが定義されていること)
     * @return R 観測値の誤差共分散行列@f$ R @f$
     */
    template <class ObserverationFunctor, class StateValues, class ObservedValues>
    Matrix<FloatT> correct(ObserverationFunctor &functor, 
        StateValues &state,
        const ObservedValues &z,
        const Matrix<FloatT> &R){
      
      recalc_coef();
      
      // 偏差分だけずれた状態量(シグマポイント)を計算
      StateValues *state_sigma(new StateValues [n_a * 2]);
      get_perturbed_states(state, state_sigma);
      
      // 予測観測量の計算
      ObservedValues y_from_state0 = functor(state);
      ObservedValues *y_from_sigma(new ObservedValues [n_a * 2]);
      
      unsigned n_y(ObservedValues::variables());
      
      // y_meanの計算
      ObservedValues y_mean;
      for(unsigned i(0); i < n_y; i++){
        y_mean[i] = weightM_0 * y_from_state0[i];
      }
      for(unsigned k(0); k < n_a; k++){
        y_from_sigma[k] = functor(state_sigma[k]);
        y_from_sigma[k + n_a] = functor(state_sigma[k + n_a]);
        for(unsigned i(0); i < n_y; i++){
          y_mean[i] += weight_i * y_from_sigma[k][i];
          y_mean[i] += weight_i * y_from_sigma[k + n_a][i];
        }
      }
      
      // P_yy, P_xyの計算
      Matrix<FloatT> P_y(n_y, 1);
      for(unsigned i(0); i < n_y; i++){
        P_y(i, 0) = y_from_state0[i] - y_mean[i];
      }
      
      Matrix<FloatT> P_yy(P_y * P_y.transpose() * weightC_0);
      Matrix<FloatT> P_xy(KalmanFilter<FloatT>::m_P.rows(), P_yy.columns());
      for(unsigned i(0); i < n_a * 2; i++){
        Matrix<FloatT> P_x(P_xy.rows(), 1);
        for(unsigned i2(0); i2 < n_a; i2++){
          P_x(i2, 0) = state_sigma[i][i2] - state[i2];
        }
        for(unsigned i2(0); i2 < n_y; i2++){
          P_y(i2, 0) = y_from_sigma[i][i2] - y_mean[i2];
        }
        
        P_yy += P_y * P_y.transpose() * weight_i;
        P_xy += P_x * P_y.transpose() * weight_i;
      }
      P_yy += R;
      
      // カルマンゲイン
      Matrix<FloatT> K(P_xy * P_yy.inverse());
      
      // 状態量, Pの修正
      Matrix<FloatT> delta_z(n_y, 1);
      for(unsigned i(0); i < n_y; i++){
        delta_z(i, 0) = const_cast<ObservedValues &>(z)[i] - y_mean[i];
      }
      
      Matrix<FloatT> mod_x(K * delta_z);
      for(unsigned i(0); i < n_a; i++){
        state[i] += mod_x(i, 0);
      }
      KalmanFilter<FloatT>::m_P -= K * P_yy * K.transpose();
      
      delete [] state_sigma;
      delete [] y_from_sigma;

      return K;
    }
};

#endif /* __KALMAN_H__ */
