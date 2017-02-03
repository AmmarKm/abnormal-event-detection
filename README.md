# abnormal-event-detection
v1123.2

開始修改testing button


=======================================================
v1123
 ** 2014.11.23 發現特例 **
 * 當 data數量不足 or data重覆時, 導致挑選的新seed可能重覆位置(dist=0)而不被挑選,
 * 會導致 k (seed數量)不足, 且 i >= d_size, 使得 *k_cur < K;

 * 應將 k_cur 視為實際挑選之 seed_count
 

training button建立完成

=======================================================

v1122
social force button建立完成;
part2 initial, part3 新增whitening轉換;

需處理trDim問題 (目前沒用到trDim)

修正Whitening的transMat大小:
	______________________________________________

		if(isWhitening == true){
			Memo1->Lines->Add("Whitening Transform...");
			if(tr_transMat)
				myDeleteArray2D(&tr_transMat, dim);
			myCreateArray2D(&tr_transMat, dim, dim);
		}
	______________________________________________


=======================================================
v0821
修改size, std去除判定
motion_th 改成 double


v0529
加入MyWhitening.cpp
加入myWhitening function
修正myWhitening function中計算wb的for k錯誤

v0316
拿掉v0313, K預設為1.5*sqrt(N)

v0313
1.增加my_k_means2
用來找最好的K, 找到後再多執行一次my_k_means


v0309
1. c_std = 0 去掉
2. c_size < 10 去掉


v0302
修正sqrt domain問題. 加入 sigma_tmp 判斷避免float overflow

-----------------------------
2013
-----------------------------


v0618-2
輸出改為全frames
frames[] cvRelease fixed.


v0618
label字樣部份不處理 (光流為0) for 'testing'

v0617
輸出偵測frame+編號+結果

v0614
不做refine
不做updating

-----------------------

v0610.
加online updating (未完成)

bug fixed:
cb_hist未歸0 is fixed.

-----------------------

v0605.
加入refine data, done. by 0606
但目前因codeword數量過大而計算max時過久

-----------------------

v0604.
z-head計算改為對AP之8鄰居+self做median

-----------------------
