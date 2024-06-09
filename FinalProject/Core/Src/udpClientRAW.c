/*
 * 	UDP Configuration and Implementation
 *  Author: GateKeeper
 *  Copyright (C) 2024
*/


#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "http_ssi.h"
#include "IPs.h"

#include "udpClientRAW.h"

char key[numkeys][keyLen] = {
		"$![XYM'CNH$/`!}=(A!QD;-!(~ZAC$=_`/`K&(VK<|D{[N{V*@T.<W@S'T.U&YO#O|@(XOK`{SAD:GJU;PI&#&SDH|/U+LV(L{V/F<'-!.@>FVD_W*D>_VEZ#AK/(<[",
		"O)A[&V>J[*&;}([S/F=P~]<C}*?$X-=F^+/FPT!<WK{%$QZ_Z:JFU*!LV<?#@U',N/J:C%|>{?T#T%-T+E~JAVO;-?K*^@|+Z/#TG}[X*'UJ[-L/SAV.XWT,W^D*'$W",
		"*;SW=.(}_Q?F<$+_YM&%B%O,-&JAUEI]DCHM.XZ&]G^}/D;J][Y/SXL._}),V|NXG$US+$;VZ.#C|-[@L=IN<[?V<}=+$IEY>RA/M'N!RITCL>/EL/{VE'#|>,'Y'&^",
		"ONJ_'S-P^/,SQ/HT_.'~:*%ZY|NR,]F*K[RT$'NUO{!=&%^<ELDRNB.;BZ-E%N!A:BNE!.IH]ZR%BKTFB$R)#&<=W|L[</)PS#.FJK!.=FY]@(AYFA@/?<#;*&G*G#X",
		"!:,L,D#;O=;&?/OV[=O>KZ_A&)KO#JTA,&D`$<UFXYE^|ULA;MXGS(X-P!;R:*,PHV#KIUH'][^U#@#~>KGW:B>@%B&'XKN^C*X,RHV^D;UTQ<W>A`.,K_SPL^O'?+Z",
		"N'X~BP?MX;H=C,`WRD!,^KLRJ)VXW$-PUD&SWMXQ/}YJ|?RPEBVOV%:{}LP#(MIJ:JZ<L&#*<@NB{PM.QOPJ`~/$K[OQZ(VWB^HC^-VYFN*#~RK&$@'FU{-N-[U}N=T",
		")S'V&CP+F,'`M#)`?)SMFU}CZ&_+VKP>Q/A+_ME&L*K,?]C_HNA;A/;/[<A_JEHJVAZ}:IM/`P+@=%]C[-&|F_}EG-+V&A%JFDKZO^EKT#A`[Y~J.T}I_KE*I=[Z<+@",
		"J^+V#@`.[UA&@G#[P;>~{'N=:C-CUW_K<W'-V!;MI=L'L:^F!$G;|UQ),HF?_JSC!JNT-P@SWT<C#CDP,CI^}AP_UC[MD%>&{X+$~L]W?LABX*O'$%'>RY$QR.TAG}]",
		"+<=M@~`+A-'DJ!{.+CU/N,&X~F_:ALTEZ$(-MCGP:+A!*@`YAV`IO}Q{C`X.QV|&[TC+-ZP]YMQ>H#KLF^P)~(_OT`{MTO;HB}U^C/;J<*N:`LK]C);U)~D@UF<[=DE",
		"GV*HNQT$?><'HGR)JZ:/^UZ$)&%(K!$'A!ORXJZ#@P$)+#'>.C*HFJFQR),=-~';=|[]&R>NRY{>*YHU:@/~'Q.+A!IRLMKBE_.V^+$,RL/FGM_/^,[*M;IKH<:G_U;",
		"`@<I$JL(>~RY)SOBC?,VU!]V<^;?`~'#W@U=](;%[-]#N+N#&ZDY^@[-ZGEN;CW,(ZG_F#>#[>`(]:VW|R[@D'+[RUKT.MJKHC]HA_XEV}*`YMI?V'#RE<=YMP^HW`=",
		"RT!|<S,#UP',(_>)NCO&K'!YTYZDS#YHWSGD(_!TYTMZ:]O]+K-@T;.;/E-SL'RS'QB`@CH&T;V]Z,#`&<BQ?H=`C]?,W&V@-R@HSGZRPXHN^;-$A]-;BD'<XEZT.#-",
		"!+?R={;N*+X)/^@%(^(-PU:KW/,S]O]#?^K}'R|P-D&`FI?^XLP*RJY[%COQI<OH~@O`PD*|<#E;:'RSU$,^;(NDG%HI;R`*HGM|/.S%W^,Y*-,H>,YA|IOD<K+@=OH",
		"O{<XD!-UF!:U|PT_/'O=$TI`%.%VGQIJ`-P(`P]TR~>=-R(H,-USK,|KG[;V<JN+MH@UZ$+.OQZ(HE$_/BAE+%YWCG|*EX>YDX?R#T+;^/;#.(TIA$-]~'{Q)IU}TC~",
		"E{KL]G`AX:AMRIB/<A^@Z>W@&C|EPGQH-U^WR)/$(DX`!XG|BW#*I~(D;(G}J&W+;+BP%QR.L=(>{IC(I/;S+&]Q~U:|RP:PLV<[]'GOBQ%[*.Z(KJHF|(]E,$?$)_-",
		"U,]:+XYS*P/Y&)~>=QR,]A^]AVDN?N|-UZ?M,@LW-X{JX)?ZK(*HLH#[JWHD)$H#,L[RSDJ+%Y|'IJ,[~^K#&X#-%]_W`>N+I`/D]*%(~('RC|,F+J>%=BD&XC!WEOS",
		"/Z`HS=,ATE}@>,?L=!K^`Y=+A*CL}Q<}:LF@/%UR}+[}O^AO%M#LK?:`KQ?|!R@.RYG-$}DKBH=A)QX)>*$IL=]INLZ^}J(MPC)~MW:&`W~E>{J`J;CZSN@Z,E>OBW*",
		"JNE'/%DZY@_~DFE%/FV*O!W^F)JWK-GQ]O+G_*'A+@P:WCS_IA,(+&Z*FDN<[XSLDRM`;H!,[AVK@*FBZAQ)D[{Y&K_Q',Z=!QD=R%F!%BD+GX}L?]_$*@.GVTO,P&Z",
		"L]>:HK/F-!]'>LUWAZ':CP.`]M)?Y){^SED!CP-W|QN>SO^'$JL:*.)T*@LE;>R+(:QUC;QH.)CK#NFA(>._EKO@-RS&@%,_-Y=X[$]EJL#G']KU`Y>?};%SY>^@V<C",
		"R>F:%?W)]:UFXC:UVWG'M*:N[;*`CT:@!E:?D=%/R%).$;]Z[MV~!<Q@&<-+}>J<XBP(?NPS~+YU}(LPQ!F?]I^@EI^S&!;QP(?XPU%B('EZYU.[$[>},_NR}TW=^:Q",
		"^$BD{OD.R$,?/>WFJHDCT[G:[GRF]}>HDAVJY/KR*B'F';:'WNV%NC*EMAR/RCF:]NS+T+%#AZ'R^Q=#FD.)U_B,^%'_+H&:;&.C#:@P:}T>X>IAGD)^/#>UZK;I$N?",
		";AE]+N@%J'A&JD*-+_'&~+SY/$OD}VK.LXEQL`JAFU+.,QECRKIDJF:]G/:(FKM_{^T#GFS)+D*KLB^W&CF_)KQ}=P;<!#O`'|(?D?%[OXNSN?[?+KORLM!&,?NC|Y-",
		"F:!)Q?!AP.(.-C<KP%T#H#<!EQKYVY#DJ[WF:.%_XOC$Q)!M}KA_#{YFMO.$E})^A<~.N`Z>XR+F~F<RYMQ_S*HQ?J:VZ!*F*E^;ST|B'.:X+|*M>)M{_^}M:S)?WYQ",
		"}`(~/'P/HIS_^!;X/]B-F$N`LMF;$Q?}@<GT?WV:>FK_X{*:(E,%<N[DC=$VSI<YCMI-$*BEZ*XLOK#C<E&L%VD#US}=$UEXH+ZY<A)D_[`O/_(+({:DLN%H^~('HK!",
		">-O'VJ#`D>;`&$MK{${WBS>POUN=G#W:S$%AN(>`#_P&.&!F#:=OB#'DY-S]U(LXAEMF:NP$ZGO*)=+%VIY)ROP$}N+KY!+EJ,UNSFGN|}<GP!ZKTNT_^*%$EGR![.~",
		",IRJT|=QB{KU&XVLNBDO=F}:R#[NMKQ>!=#$TZEI=$[%@=AQSZ>UP(=SE=IE)KRH'SC/Q$*!S=$AFVG*N'<WQLDS_YMWH`I-!FZL=COQ{=#RYQTP}[EB!=&O)ONPQW=",
		"E*=>$JSHJO'PC:L:~.N^:P^Z)-,V}KNQB[&AOCFMU;UQ^:KZAZPGE#'JO`|,$B,{Y!;FH[;AD<A)<AH+E`HS|S)D;~'*R@_.KI$N,H.(,/;^,CM*LMPS+`OM*Y+BRO'",
		"]W$~%-U&!KATO]@CQJQW,NL/;N@IP_EC`}DM[+.I*^`=PH:WV#B,<W$;^_$Y)-R$I=,VK{(B^%^.?]|;=!`-FTWLG_@T_!=PW(RK^,XB%+RD)&$*SH+}$XF$.)P$<S:",
		"DBWR%:#/AYQK?ERH*C;H@~.{P^:T):;MY-I?(OF&NS<JM`}&$Y>=A!GH';L[`:E|-CDH<$]`:P({V/=*INCF[(BKV./-`'A[@WY!}-*UMT<:RHSYV_:;G-*)*B.MKYB",
		"LB'>TKC#'J>_;QO.Y,@ZL%>@B-HOXRU-'J,%}NLXYM&'_LB{.A|X,;Q~PN[CBG+QB}^I-|%DV>(;AK~*'LN#I:&,%U_KG+)ZJZSQ:JS!$-({+B/!J]IJIQ/H:Y*TQ*Q",
		"=^%$BDEWV:~AD^M.K!/=(;ZI,C<=;Q/&R!?QA,B(OIB*-L&U@I-EY!)^;U.+>ZCJ}H;]Q^<+B<;GV@RT_NT-K+K.-!&)=!&V+%O[Q;Z_G[?X,G+GK;W]<D';{J:%S~D",
		"?=*WN/@KZK%DXHJF_>.VKQ#[<Y=RI_^V)A;N'B'Q.S$QH@VJ&%DL$E+ME<VK)}#>$/]F)C-F;OK+!<C:<R+B$YUI{UC'I=].#H>-W@/B+(.SK`U_)>`P@]+~O#?]:,-",
		"S)D;R%&WU]R&PZR_,W-?$EF`UPCS%'_X:+Q#*HIHT-&([XFQE?EI_@<YG@&/Z&Z.EGK*A{X-L|,`FK|V&^+)S.^ZV>$!D{PH~QS_~?`!`I<~MQ`/H`D[C]S+]=YXK:Z",
		";)=YOL/IHKR@WQ|?)WCLISW/>;.ZGZX*>CUG,B?DNR=~M'J*?@M|=,@<AMZ)^/]BRIUK&'S$,XH@=F#T,TZJ@PF>J.{[GAYKH?YU#NBL=ETG#[/S'[@KIU]W*HF?-KJ",
		"|-*,I(V-G[*}]ME!UGP:MNZVX:WF-+KS'/D+Q&LS'+F(DA^&MP;^V(U;QRXW`-F')ZG_/IZ@|<OA~);YO=TB}&:R+,']$GR>H'J&R<*-I!^SF@Y,H_}F>R]B|V^Y.)@",
		"TQZ?<G'*!B$`MO>HV'QEWGX`)DLN<QSL]+ZK{F,O^'/_Z.$`H)&<LP$[Q+YPR$L^_S({E{@J,B*@-WRGMJKFD-KMQ={I!R{T<L!.}EWI'V!]LJVG.V`ZB{ANZS@?YU@",
		"C&@WQC[IK%>;'@)_KD</G&!)K?=%T#C>V$D-HWDU`_B)_U/[NI;SX~?$)][#;+/,/Y$|`L,.@P]!MP)F+`D(&R='T}.;U]>.?Z=G;}B/!AG;H<~#EI_T(A'{TMS|>~}",
		"TR|VFPHCN*VA&)U$S[)Q[A>?+<={`?ES.^:Z#LB]/S,O&H<D-)*)@#ZY$@{&NG~G?Z!LT@/^<YR^=/HMHB/}C:M~U?RKJ#{T,?DX~E_^GCU%I$WE`}&%RGWK),DK!#@",
		"^`></$A`,$O:_UXWUPCN-S)'E|D]*]${SNM<!@.?H`(O`K>;FUOM{%QFJ{#$NX?+.R$,Y('X(~I:#^T[$)/>E-H*>O$.Q{GK}^'ML&@J.+>=[_$L~F/X_!IN?K@AK^#",
		"C+>Z%(M^G@#G%!RYB?'_Q{/+E&TS#.?]?THBUXH=$O+@!R^]LK.GNOT)+{R#*@X_X>PU{<YFI;Q}<#F};^=ZQ:ERWRG-;@'>[);<AE[>(O_AD$!W-P@;|#?G-@,=}N>",
		"~@)[TI*.(;KFW/}<'DTC_I|H'?H*D'>F<KVQGHWA,I?T_.I|ZA&C,{.M${G['O=E?$)H~K[OIP,{FI{KA)RJ/>MLFLT<IL~_K*=G#+TL}&:JK;!CX%Z>N~P#[=HB{T)",
		"]{BNI:B?K_=,~Z(=KSOJKL(=^V_|=Y:S_^-~>J%$S:O,(H|E:@?/%#?$=_~+H{EQ/)VIW=,&B/D_H`C^#_K(Q_:-)UQK)J,FY&(.^,MY)Y-&;*ZY){NB-TN([T_<I#O",
		"+/DO#EDMO%.$?FNZA*^<T%O~>I*W<TS^J!%BI!DH`Q|B`LRXC'G;)%WMW;^`YTIQ)+}LG%=+<B,`/NFB`]YL^$.(D$VA;LB#VS^<]-OQY_J>~@CS*.C%I_G@ORM?A$!",
		"B@>;|,^-OWJ}]$G`P:.>CH_Z#:^IW!U`<:^~=N(#@;:R?KH:&MY`$]I+!L#>G'@B>$/]-DSKNHB{Q%:U$)-+}O&LRVCM#*,'E(WHJ[`!EO!JR<O&TR|.&IBG;!`R-`Q",
		"K`:RJ?A=-OJO@([-#[V:>P/=YL^X,_AUVA?IOQURH]KFP#(E'!FV~^OSN<,`E`_K|DR%W*K'`%)YS[C*-<U{MOLAKZE}B>-^]Q~]/DVM?Q>G*W&}^JEOJ=])AU#=QTA",
		"#RT:+GQ@PY/S$=FP.([K[{HP<R&$_'W'?M,JOWI/IVM*EG[L{!NW<VGZ&WLN^}B)TU@+CU(C>.N`;%#`+>(_)S#:TZL$#F:&.<;ET?HPB]|_R(U+WNX)]=DWZ[{T<]M",
		"!EYS@,ZEL.S&#|?:^-YMHK+(EUB%RP/@QO&_NOSA_ZO^DHRB.$S^,=?_U;MJ;Z_=X@)F{I.S)BC*KDAX~XW@`|X/TWJ!|)YK}+Y*RUH.}H`G):SR,^FGYFLJ?HG?,%Y",
		"-@L.>N,ZBEBR*Q,JE}>J+KRHMSCSI~&=V*$]<HF:`VEX?M=%U+H);NW?QI&DUPC(D,#PWIYE[S?S)/F;N`GJOL:;Q#CGUT!)&}L@G*!W:_FG_R>P*P?|QH#/PONYN.K",
		"~Z+$T#[|YQC'.W@LVDLPTE_>U[I<=+_)(:;)HV)SW#T)D'|L'BS!R[)'&!ZYV(E^}*'>J-*P:L$;_C@R*-%_!?-}Q%VYU*<.(IB(FUF(DZKI/%X/W:V+*]R*!L+-`SC",
		":?<.=~WU*M+V+P#[$V|/<%?RX)!(~'D<*N?`*|AGDUD(G{`?&`-%?K.()'WMI-DLSTA_+E;}CAGVUHIZ^-#X`G<[!'|RP&VI?M>Y%:LE/;%P;+M,Y/DC&G]^:KNSF$`",
		"'MHA~KOQ-JCAQ:}<X=^)PA-ZX,*DQ&/I&,PF#OS/^T'VI^S=}%+$TR$;$IWRFLXY:R_A:XYN*H][J>J?}:O]J{P^GRPF{ZF@Q<'L?QYV<Q!RH+NX~)*;?[T&@_[&IM}",
		"CMCE,YX(.*F$]L|F#V(=>@>GV'(D'(.U/<OGQ;S'?QM$G$=&=F&(B:NFT<*,J-IG=_@G*$&$A&'KP*,`VU.'T>X*U#K@PQF|CGZL:H'W|N^#>Z:IQ`J](?TX}H!GZ=K",
		"%N/!]ZCR~J-|O^'B+ZS$`{Z-|^G&D`YJ`H@P[|[P&Z|X.I=:`C<Y>A%/_&{)]!]).BE=Z]_Q!.JP(G[CZLR}*Z,:($@-|DMK/[ZQX$V[{B%EB$B:W*P('/=Y{#!O:%A",
		"WQJ(S%NE.G%;@P:PS'NEY(%[#S>S&)&?A-|*>K>*+}{K.'XM}`VO<^T%-,BL|{H;#|{RLS!W=FD_GACB$^>P[AQ&%_.F:,J,^XQOX-:ONA+;@`YCY?O^AU]R<+QW?@'",
		"@PF=>PX]R,.,'LJ}K~CVWYRUD.!YF,`!KPKVUS-;TPMLKA=)X@X],$])_RAK:=!}LM*$][UQNQ,Z&^`/|[H_QAYO(_FY&+/(J,<CHY?B=,?])*W;$/D*?NGFSG?[>U@",
		"_K:S/F<^JV#|^U~|XT#+B,(EGU]N.QECW%VJ/R>TO(Z?R;Z&DRS*KCS@CHS.AH[$*UMD<TU)CLDF+.*-FS<HY?*'Z>)ZUZ'~,CR/@U]/#'+C>IU_^#%<TVJ#_=H?NF*",
		"TPL&G'~:;)%~+^|X.UAS$`TZ|YGXFZ.N_GY&-<B:,|(%-F}RT|,`D~A:G)/LJ)&-LR/KL;,COBV%E{C&UYVJ$/?N]{%!](|/CGL-L_[Y]^U(_!+Y&G^<@#MO^GER%(,",
		"A(KX+HL`<(@TM:PW)FIU&/H-@U?^K]`ORFO[T%C|^O>F&|M#H#H]S*;C;=|'.ULRXFCTH&PK:_.|IR'I(Y!@'M`R_NFU}NQIBO-HT|B|AM$)/>'=Q@&$*:UR?>A)-YW",
		"&!+%!^YT!-&N>L~)F{Q]`)X!CO+G%;VXW!NI$F)CB!PD`X`']P+B|F|KCQ_@ITH&'_>Z/%(@!K>`;Z-]W;D:IS{*-G[#`N(/_ATO}BT+F'LDXFAY,UV?+L=!HQ^`]TV",
		"/A>D.S!+<J-V-TSX_SW(HY:CYCI~ED^PQ-'G#S`_->A+BP/Y+$,ZS!/(V.Z:F|Y#R.#~_~)'^?|JGK'<{~>`KH/)LUQ'L!_IKDS-*YH@!^L-B];VD#}UOEF,PMAN.HZ",
		"'Z/:]-NG/A:)IAFAEL![#YBZQ>M>WYMWNU}@Z+-]}*,@=!,BST]Z$)XQ#/SOZ!&BM+?EGBI(/F~Q*QI`=C:M~_X^?~;PIWV-(|F(NO:-|$O$[;Y-#OGHV(I_VIZM->`",
		"[CV^J&Z!TELZ!KRA:M%&!UHA<ZGZW]SC.YC'{=,-T(%C^VS(!+-/K~X<#@;/@?}|PQ'~=S}WNMP[,UCPA>.SIXM!$Q:SXOU]Q>&GU^UB^R{;I)M=:V'FPI~+E:[/^/>",
		"#*'D?J$.IS,]AZ+[+Q_*C.JC)+:RP(].>@JKAFE;NE{]=?RB>I/K=&O%_Y,C/Q?)X-|U}SFM:!F]:/K&<L{)I(<GTRJHG~|LJR$DWKMKBJX{/;U/S!*,%GO.E{JZ!(R",
		"{X+F_#M=~F/%(HDED<*~N/V.:?J#>,KCWVJLZLH%!D[.]_R|EK.)}TYASHQ|GEI[DIO$XBZ|@-QMS};!KH{N^:FU*`KU{&{:J;VLW[-&*:^%`>EUDR[LC$>J:=TJQ_D",
		"V?M@}_HS/#LI;!G]E>JB{;CUSHJEUDHGJ,{[)[OD/%)P;C/,](S@W#/W=$WB`>LIDL'<}CTQ$Q$@%^W;R+|^T$&,W+@UM/:<=;A-T!I%`X|V>=F&?[~<{Q!?)NW:U}@",
		"VI^J;$I#$AVGU_J>;$T$ZSD/)G/?JX.%T)}FJ`-MKD&M[~E%Y+C+JHKBT%EIS-Z#(N}.*[S)!TWIA#KCA{}'?-#-#BMZU^TC)L=$Q*S?UE)YVAID)[D/YT!(P)(FX#F",
		"&J<%-XM,HS&%ZKM.^QU>`H['`>=CB*)KVI~Z]I]SRVMK:`W|:@I:&[X}EU!-;:(K/<*[?(PYC-<'X^V'>F(['.'!=]D;SU/.;.!#WDB|KAG(H^@&.B;<BAJLJ<XT>M_",
		"ZTBVM{F*$EXY`EG+F%C=-Q^~Y_}F]W?NR})'F,K}N*M'X`A^O)#'PJ.NLYQJO(,%~&<,&>Z&@,[|]{R.;)?I.(U,;?IYN]SE?|[L~K'IKTJ.[?E.U&}]+V~%;/MJ</&",
		"I%D)(<?RT+PO&}*O.V<-AV`!^SKL:J&K/,!HG-?|~RZ'$/U@D&)#.MWXWD!}${.FN(P:&Y-/+K$SR>ROA$L-P^W>]'G~D{O+G%$QT_T;']BL+J+/JBFL)~$,`WO-N!,",
		"TM`+!+[CPZ*X&)`GWDFI-WJF_$H%>_`XU&KZ.-T~)^NO@>E&`!U_HO*!}'}-M/`@U#>U>.O:V};BIV%#KB;/WJO-;+*-YC|]NB`?>.CAYL#JILQ.ZTRVN?R^=<+]|U_",
		"W];W/H%G=MV!Q=JGKZ=~SWV:!P_=T&'&#|;M^GD<+A'-D`AID$B>EF>%B~M;^XEKTK($QM*XM[*TPOS(P?C/#!M{%}Q/>ZM)]EL`}N&:GHYWYL]J/U?#G(+XJ~.T-GM",
		"EYM*=I@+T<CH).R!|(:E[.#_RTS~^;<BL<M^)'H$A_+Z?!+>&LX*+/ELV$+;KZXTF}I<@>,%+F#=+K<_I|GF&|!C(:WXSKTM,~}O)'.$T'I;>E!M<`NG_K`JSEPUSEQ",
		":{E;X_X@`[EI=@/%TB.B|O/OBD*A?~!W@^MF*=NYNU;^RY^RAP.R?E@:&@CNZLNVSF)-#D`BK>/%ETEY~VMK(/X!JG).T-=Q{J<|.?Y_*I]:H<(!=<_:>=?X#?{H]&,",
		"*S)E}`%;T.~(K>=>Q@<?:D$N/.>)?@Y/!YS$.HJ:?}B&N*=RK,X?LXP!N.FO,?-!ZD*V]=CP#YC[&F@,N|%ZC!{U@!=TIY.'ON.%VED#}H]MSM!M&^IS?T]+@V$Z[`*",
		".NI>VF=<+SP'G>D=%FBL>BCV`[Q<)F!+%WH/U~N=+'?#FP/DIA-S_{C.X+X`J#$_%@-#=$#}@|(J=Z^;U)-F:(}>D+<AP`WEF|;JRFH?WPUR(J!RFPBRI.C;TE.N^'/",
		"PC@P;C.`[A}$WCY*.Y{R,S<O{N!*[_-ZJ-G?_CZ/[:$_>VY'>M(!`KCX.UH?YC}FJ&V(D$/^*D|?SZ]G|X:*N(%/C>LDG#D$[/T'|-]K(C.[QKS;N@G?/V;,~[/$%`P",
		"]H{OVXADLAX&NR}[SG#@V#%NEN[-+GM):=IY!STVDXF-^:J]K,.E,(UX&M@-@JV+TM)=FTNQ[|A>~)]-P[JDS:%B#U}][LBJ.{]H!>T<X]H[/(;[|,$GL'%&X#G);!Q",
		",%B?KS.OT:A<LT@I>UB'#_G|NJEY.DIFC-CJE(P>$SJP/CF)ZN.P%IE<PSI./FD.%H<JYE.]@$H?$LH$X<@#P/LK,`_:>:=X@#W[|=@,PC|H)PB)*O-<Y)LR(IW(.TU",
		"FT*N.X:W_A-(&/O(VUL%.QWZ@%NA?K+F]C:[#O`BV.?=LQ#EP+|U=!`@PE-P'CE:*J@ZYB`L+IQGJL<FA'GV.D=QEZ|'[|(P]*&IEK&K.[Q+JGL|?VS~ZO^[H;IM!VD",
		"CN*(U&N&_X.T'_?{)|,=^D%I=[BZU$EK-HXKD{R[M[;UK]!-E/,>;S.S)>Q*G@,ZKG?'L;,I:;=)?ZYH_<VG<*MH/I%:-<F&`<%Z&]>&<[|?GRVAWN/+O-/D'!#M:@$",
		"<L-W>@&OC-H#Z)C%:>HB(X,J,C&<T'H-#()BPX`JC.%=-^T!)_)VA,(#:!ZK@A!#U=#[HEUBMI;,)O)DXT'PF=I(.!EI>&R#}ASL#U=GH,NHJ{$-/HWMN]~Z[JA+LH.",
		"YC~=+{IYGPW=H*+*}AHGHD`,~@[?=*ZX'=SKGIY%'<LM+H.?*^~L=UDHKJ.Y>]DYD-WTF$A&GM-U=~=IS{=){T.P?XCH[=P@Y?#-?OM/-#=[OE|K[|_;()VBR^*:X^R",
		"%?$~V%JR_]O/_L[L|J)Y`R}!~GA!AJR]MI_,_#MN;L,)FD~SA'D%A)C,I+W-QZ*Y|ZAIYQ%`)=P_/O:-$`.:}/Z|$)!{OM{G/&AV*WQ;V`#NY-=U[%=LM!]'NJ:']R(",
		"XTE`(}*=`$/+'TZQM^AM!/_:]>=LGWBLV$?_/_+)[!,*JF}_ZG^,|OI<}Z$@-^-?E_]MR/^FBP{K!C-=!S(?V>U,L/=`@YGXDCMOHLXIH)$%'/E=[ED!F_RD]X_`F}L",
		"`YT|&BY',<'/K'C$#}H/H$+#X(R]FYQ^X*^)HF)]ABC+$UVANQ>V=${}*&[$W;YF$IJKO=K|)$K:W`X:^_.CMPEM{!,)?M/H}`:,$<G+O`^RH@]P'`%!J-GW~|}E!/`",
		"*`N;UF[>V=?!O@.=V'&'+@&US/+!LRW$,SK:#)U/Z*,K:E;(TE#ENTQ_*?FCBV(`_/UDW'GTA-_O#SPE`;@Z_@-QWFG['E/_EZP:JNYW:XU-}&Z>U`QPX|F{O}/#<[H",
		"YPN:@J|W*B?GU>][.#C)_&B_.HM-?;`.J:~XL`C,!;&_A$=;J|]EQCDLQZ(Y(&:JKVE'Y{A=`?$]AG+-O,V~*[S%Z>.Q)S!Q%X|~C'B]N!BY#OSX'W-`.^C<ISF)K=[",
		"+G<OY>GN)OWPYHN>^U|<L%W^%,;FIY>,S{,`VJ*G~]:$J>.=.'=,.U|EQ^IL<@$^/;B+Y,E]:>Q=+>?M@:F_FS%=?~F{$-*JY@%U.`S:EJQ&QTW};RZVDOJ;(YETZ<^",
		"D[H.&[+X~B(ZCI`^M,&>:+D=(>VR!H%(<^CX^&>DGJNZ)&F~$:|G@?N/^|+DT:]>BQ~A}_>_&D?>PVH/-:M=V'EJ%/)$MF?T&X?[NLSNB@]DE,}S/E#M|D!L+<]OUPN",
		"<=`UPB?Z#AC:>T,@L?+KB:^M%+CK-)F=*'@W~_$X%D:+A|=Z[=M<UX_QP/KPV,%>V.FNIJ<Z%,!WB_@F[AG/|XLSQ^{G_{KLDIUX&K,T-:ZQ^V|@_LR#_BKOP~+QB^>",
		"(D=(!WU;-GQ[E#D_NK?`XISN&.JZIWJN*.E>@(B#^}-RL_'CQ-(Z>*#AL)H*<J?,|#IDGE=OE(QU^#AV.XYW{HP'L^/%];~LZ*-F>U/?TUK@[}-@=!PLU',$MNU)ZE]",
		"@TR!+;{[#QX{_-:G-|XB['J!$E>Y(.[E*'Q{N+SN&%=FJ]C+`MU-AGZ<SVK[R/YJR,:RSB?D{[ULA'BPYP*$=IV^C}V[T)NM+M!=JA]VU/$!CNS])V>X<>?C/QVHPV@",
		".)}%KHV!QMZ/W/PXA?#&NWMS}O.GPA(B'P.!#;Y[LE(R._&[`XJ<#:RGKVH,DG;YZUA<Y^%E(@F}BY$(D=WTFKRO/~ZY;N,}?,RHE=`[RQH;]&:R;/(*Q/C+N)GKZWV",
		".{#UWM-:;DHES<O[PT('QWN>[K'TW@Q_RM=CQ!'E&T.DXSYC&-?RT}%~&X.#B#FCTRP}NM)!'X-Y:|CA!C{$YWUC>G]&O`OYC!;'[EW:J(`-NX|T.,E.[:XS^->WH-E",
		")POQR@+#EWP#_)!{_C@LK,~G^=)'OH<V.Z^`/=;[GI@_ZP|_=^.T]KYR'*{-U'J{>G(?AE!BT:WNLD]SN(OZ[EZ[;<ZBQ.G;~#]UF?;)KM@CM&B);-!<:WS-YKI*#-J",
		"L<][>J|=}/(#<KDENP!]%D@}$S@ROXSR;~$;(!;=O?MBTAKGR%S?RY)}&BU/YP(=MIL]/^$OAB<B~F&=|M%ZRGOK}N(;,>:,R[MZ=BZNF+DL`NESD=<HMS;K$#+]P#/",
		"&XJ)>B<!QD%R;S_>JM{RY`Z#S_V.J`@}]RKS>HON)*QE:+_^,/U+#{S;|U^O$H-I)@JKY;H=P?%JE>}O`]$|B%|^:|/Y*`=O:PJHBE}(?WYO'P$;*=:(FOV<TP[G<M%",
		"+?)/_?.%/(-U~H=`PN?CV.,:`OM()%&JBFI*I#WFIQ@&_*>H!&P*T<}>.D;MJ>@L$[O#T%-*&IZ|~`>)Z?DB%{P_:-I<(|G*BLU}+G&U/+,FC^Y!@<OT[`B?Q[^=[GS",
		")!DY+`Q|-.]IHL$+CD()'.*:[>#^?Y#-_.BV_>|.A&NJNW&~(^OR`%'>Z&G~-^-MAH*M)M=SQM@RSECT']!S}]KU=*U,;K])LI$,NTC;V[PF'%AT!E,#<+A+T[J,A:?",
		"M.Q>M{$(%BA;=Z=[IWT$RV-AI[&YWDW!MI`ZJO]|F,S[/|@%=,HOZ'A)()$X_NBXK#O=IC,TASRY`EI/Y-ZG>HZ<I'=TVATS`B;A.VCM(+.T<IC,.(;U*`%FG@MO!%.",
		"M&F&@%KMS#`,WA#:,%,VX[`_WF~JK^`P'CTQV<(_V@_'P)NG:'JYE$`%:F<W|,;!<;C<FY$@>-^?HJ>}X;)]DQ*|D,|]=@>K%L!J[~}+>B,BYC]NUDIU(T&*I:P_D#Q",
		"/V=-L`U_A)H.DAZ]E=]Z,},C]`#I%&N(E:P_(N([E%&F^RY?/E)M'.*TP~T;{QGB;IEP}-$/FJ*CKW,-W[I*)Z,Z<SH|Q&D$V/;C.[V-JWH^(WX&!+OR@LZI+E`;RN:",
		"U-+HLESVX=E=;T&-^QYNZGK)`C(M[S-ID!QE)}D&>PQ<X:BL=*B+&UZ]+/ZBS;(OS'@VFHP`WP*+BFEM;(KGHD/)*VY$=EG/I}V*R/R@FR-+J'@{_|JE_SX&F#!XT@Z",
		".`AR+/JE&`?=/T)U(I-KHT:#J+{+L&DNM%{MCJ&~L)IRS#!{$-Q&W<?_KQ(HZ^}RF'RYR$-X</W#)>S)#&K%`KO/@?O#@%$?AGO_L@T-L?YRN&.CL,)${W|]$*:&:|X",
		"VF[SA:#*'+-${DO;G&QP?WXYK)T,K^-HIQ:S]W'<M+)|I}<{#.Q(*:RKS@CEDEKIT>RITAF}@XZEAH~CDS'YZ&!RWL'I,^@!>H=PX%M[_{DVJ<X#]#.V.L(A_E-JX'`",
		"KQ'C>ZC#*:)ZQ$:NJBQ^}{?F%Q%WS|B$LF-}IGE)E%A[O`<=BD}Z(^@)'JQ(~X[OM&-C>J{?,+U!T@Q@XB',GXSB_D_C.|L?>%`O_Q@U+XSNB!?|DUAD+J);*Y}~Y>L",
		"B(~%~{ZR>^.{KFSR-%AD^ON:$BQ])|[>_^G=$A&)Z)](K]X_U^W-R?C/FPNE?W>F>|%);%SN[_.[)]:Q_CX`M,^{J`*I(NF]@+W}^%*=<:>R}C<L%;.~}Y^UG#,K,I#",
		"_T^E,EIF=DV*IF>OS:/UE`)[+TO,P>J'GL!_R)C?!.E./A?L^@SVC[`_H$>@:ZG,~/.B+`VJCA}Z$Q]HQ`{#']HT$&%C$|`N>CA/OTUP.;A:F]KJ;OJS&ZCS.}R/L=>",
		"J&?#KQW=TS(@J%SP=S&|O|(:/.YX+D]/:CMP_I(IOVM+%Z'*=JXE].,@](;B/},CXV('<^QY*!(?]PX.RKFR&W$LE>]VR#],`HIUQ/!O&S:Q'/N^'%G([+S?&>N=.(`",
		"BH;QW[_+*%^H*!*%@(O})`]O&IM*T^L'(UJ)>(#|PM)VC'=+)%=+/D`?-F=/]([ZLC|U>FXB{XB)O=EL{AUY?,I}>'!^BH;MIC#{O+IOAG^AM^G<ORC~(H)J`?Q?=)>",
		"`EI]I@>}).=DK@YRHV!J/ESG<%?~!_>J)KC!)@Z#:H,FVT[=ZFI=,(W_&O@-XM%H);Q}M&NZU`O#`ZWJ?&-H@]`LHYMT,{E)+(BY)&ZI<I?DW'=';=Z<]I/~]_}>=Y]",
		"^'CR/_GJV%]_%CHC,B[B`K^TRDIY>{+QBM:-KOB.-MP#*KN#L=T+IHOER<>$H=W$_&`W)-@]!?<@&@S`O]UY[]!OTR]M;YKTPW!]OUX%INJQ+$#S[YR['MO'E?UZ`V]",
		"B)WXR&V(DH{$=]C=FX(!S^]!;Q).D[*JL+>#<[?I*XOPKX`NE@KN(*#Z@E&GBQ,M_TMEYQUWX/MN.SF![?]MZL~&!T)-`YIW:/&H'.Z[K+PEJX+TK*~B[HVM:,ZU+PL",
		"UB@:'^#:(D[!-?V+>`H-AXZ':)K+;%;@-C,(*XBYAOK;$LCN:V&UZ'&FIN+NGS+</?Y)AV;QG@A`('HP.ECHN(`<W}*?FMAGB=&PR$WZRV]*$@`#-[+~`N.D@L]TQ.B",
		"T~{[R:UX|N|IWT;('IPEQ=&L=Y(TB%&/PA?*Y`JDZQ_}U*K:/QH^H%Q],JX&/)~ZJ/XH(P]T,/L?=ULEQ~#-KT[ZH|MT<*}#A,AH_~'<FND[HAJ@*H/EQGH-S$%#/)P",
		"T;R#<F`#[;,>I,M^Q;!}N.}{:T>_{ASB_.B~;`:/`R?_=HUND'+*?(-I_];U~C>[OYK%+>NQ$.'{(Q$F*MN~COG*VXJB'NWJ]@F~-#|AK~ZB?$I!<BMVW~>=M!}BK|I",
		"YU[TZ$C,T-,H}{I%(MH]R)HJADLZX;>?GF/EZ-?TNM)C$|%}Q,`E)^X$M$NYI+NH>VA_$`^L%*C](<P?@)-UCWH%W(!%@%,~JYMAYB|%,D<-Z;KO][K@L(UQ(;'?#J/",
		"%KZQD:(-EXMFD:N>$HDK)G`%.V@]GE)W*Q&{:,`EAI<}FT!`&Y:QF]&KDE,!FY-+EF:{.X*A!_RTK>]#+/Q:<C@PO^Y:DQG|EYUQ(`X@O=-]FQE/F$`(;Z+W>|_B#R/",
		"%B>)}%$@TZI<Y|`-}?ZDH^@LJ{E#};=&Z-;OUWOKLX~@NU-/E[J`?)*^XOS?V,$UXC>HEXW~!D@=?ISHB[;,YUQ.;+V_IHT'KG`#!DVH!L-<)LZMHTHP+?-PUQ*:!<Z",
		"$YKH=GTE>R~+FU+NMEVE]TI[!.GFRT~%SDXEHPTGV^W)BM$B?$!|X:<F]#'J[AO#)M!CJX:RWDL&[P+{<!)C^,]OM-.L$W~^X>P@T#~/`:[X*X>&=%O},O;-RSTJ]S:",
		"-]<.AR}/P{J@G#N&TYHSUM;/W`SK;W.=@,BC%=Z$[BV=%;:(`,C[_)QNHYN|R&I-)%N[&?~K[E}:#`,>T*XN#NV(EG/-%Q<E~M%.^B|&+:T`BU$G}$</WS&(MAXS>BO",
		"V)R({LES)ZAT_HI[$+R{C%&ZFAD>U~_*B.A%E/<-,&^B><GC(^KFEC{DOE_[QT=+TUXAOQDZ/?B(>I>A@B<I]WGI_F#<VLYR^=.M*ID,ZOGH?:Y$@DX$'X@!K-TUF_J",
		"B-(`F`}?&'Q*DV,(FQ`C{?BWB#SD}I!PR+@:S^K@FX</,%:.W]BUAC@/MON#Z,TA`%`(P*[.~[,EP^.QW[RI[-%$)=O-{VB$OJZ*LEJ).W.LGPG~CRKUOD^E:,^!EFB",
		"NPF{(GNF-Z,ZG=#J>-*OL~?C%JD<]~H_*=A.MC{K-,NT[JXZ>NWK^.@R?_'D_%JMWB#W{~,WXO,<JCD(=HAT{&<],|W{VI,=['X%TF,^'./SMP]'P(+.L-E#ZJM$|RW",
		"YN&D[<KD`Y]?]}?;T(E{L/)FZ>AJQ@H,Q|_{`SK!Q/`~SW[-V*~.J`D[FT[KLKSBW!H`)},BU'!L];!OS)BSYQWKYU[K!L):_]FI/^;O@M&Z=`@ATM(=AM`_*XE'Y_U",
		":V?LS;'NG[O&+I-DLWN?{TD|N<@C)J?K^>/JSM#!N|[KF_!D)-@B[]QB)%E@[])].XDVZDEIR?]$`!(EG`=VE)`WL+P!U(T()XL[+/_Q@`}.).'>;]=LDE&+X/&=!Q)",
		"@M'LZJ)#^ZI_A)[V.!&O`{-Z.S_?XV*`M)N)~^@<M!R]-DRQ'<T=L+{<|B=UWKM*![ZA/)>`:IX<Q/.@S/D'>SIH<W:$#FX}Q#%].TLI|/{/KHV,X?F=*'F.}[-)R-<",
		"W;|O<YCI_+H=|PXG(YCR*SFL'C!NW`DBU`Y@S!-:<P=L}^S[S*`XAN}>WZ,-GY[N'BCN^?*HUB!(-NYS_NI.TRB+XN!,!@>_*_)OF>+$?[YVURF;,]&+/Y=^G-,KJAQ",
		"RW.DHKBCQ+[=COY<`H'LU(VWY/<W<R}W'[*!~QCO,J-$HFQ!$'W<WVA&?G,L|_(^K*J(*MVHL:SZQ@A(&>S>HAQ){%Y}W@^O)(DVUSI%W~A!Y,%M-{=P?_*B`HNW#WC",
		"}W*=O_T+Y-A-=@?@U|XH_,UP-?R,)_<QU+EB)&;Q$!RJ,EP;PO/[;?KFT#]_./G^H@V-MA[|HW$`#]E&Y^N[&G>,]_F>U<;QXTMOG(LMEWK](+M-_G;E:?Y!{RM}[N'",
		"!<N+E%A[BH'|}-,`BIPMS#^Q.:]@/>;,@$;UI]'HXAP,TQ,CBJ]N%{!@I&INP}~CD)H!#^HAUI%$GU=;}NZJE:E?<LI`[<P?AS*+W-C@G'R$JN}A%N_E<F+WNI[D%]%",
		"]E|`!>E@]JT(UP*TE&=<J!B&=<C#@[G_S[D)WN`-=GBL)]CS^,J.;@TJXH/,UMA+T#'TU_~O?[L@=#,.`<&P~(@%?`T_P$F#~Y{$ABQN=LXQ^M@X],_,(}JO]BZLYQB",
		"?E{P)[$J}<?&;><&JWD?V<YI.#S,-*LP(K(`/.V%X/`{CZ)`?S.F#U'`~,H{D?MT(U_]{_MO,#K*'RHS:ZE?IVXBW@~<XBK!~%E><I;-<MRMS$LZETGX_UM<,D!U>?)",
		"Z'ZBYF@P&R`Z~(M+-_T'U+SI[MOP+CPLUP-J<?,#>!TWFJ@ROP]?;?:{U;<`>K:XJ{G=C-RB[@S+KIEMIZ~MUW'WFP@.FX*Y*W=;MTP>,-<M.D,K&XKUZTG{JQKHM>Z",
		")KU>V(LX^U~Y!?#?*&<&#Q)&@XCLZ<O?LT/{@-UR[ROQ&S~-]AQMGY=)?YV*>DV+%}#&YU&I(#N[M~&>?/RMN>RV%$!DM(G]=,V#%*QH]^~QPJTJ<]M?%UXQ&I@;:'K",
		"RE|;#AZ(M:|RO*IP}I%H?OZ$&[`P~|:*P$.Z:X-BGWOZ_,D@[]MYVD]CZ>HL%$S+]%UW_E$P$G=BFG`)R<*V&/R)`X,_<Y&`|=EPKG=BN=]|Q$/K@AL=IB*YK=.#BR<",
		"-|J@EKHY#Z?${-UA!M~_^+-B};D}/?,>^+GBXRBQ;]Z<)BT;T/LT^F$QFE>#*=(V?B,B@:/J!}CF%GU.&,ZXE_>}<@A#'+G:%!YVGM*AG;I_H;$_%D'B-MS$^(C`AH=",
		"B$H$Y=-F]N%ZC{DZPA]_Q$]=:MFK+I_.*+QM<LGM@KE#;,-UB;>.#Y.OP=Y%OI&Q}L>!S-[>O-R!/XDX@*XDXRH`:_];]IM&(>{V;/Q{KG*%YPR^}-$WJ;RA_[!K/[C",
		"AIL>D<R:^WDXRZ}|L:GBPC<W$,+|A+$@)U'SF@$AN#O[F:Q~*;FRC'-W*$U+E%GD)WH!C:Z&C~[](&KDY*L}KCFL!Z!_D%M%)*'-/P[^O#A;!']<L-GP=-VIX)@U+WX",
		"E=T+T=@;J#R=)O)FHORB`<NO=},UNF~P#_L,AUCY~TU^>L_]RMLBX|'+RMFE=>RJ{]@|)-.VOIP#=Y|AL}BJ{%`^]|&T{U<>;)|~K]+HUB[_J`!|]U{`QGHSO_[$}H`",
		"%HJC/:/F+{H[=?T;H:>D*US[WTND,(,ZOAJH)]!DT`+VMT/ZJ)*XEQ=(K`K%!UWQ>.}AOW(K_+SQ_|=:URTZER-ZW~TR'.&$&R:BQS@GTFI<C]P;?>=&R#-?=ND<&>^",
		"&UCX:/JM`*%!'-]Z'<BFU?&RU'PKFK=UWRNMY^,?],Z]OK>,FHB|+J*C;J@VN)DQ|*[<QTC+R?>%Z-CUM<QJES;@SAL|&UD[VL>V<S-YXIPVB-GWY!)(Y;VWAE'O/!X",
		"-FJ!FS-IR$O)@PS?VL^'ZAY$M:NDS*X%G}!=I/O}C#WP@Q?YNLR^QI_)T#!JHB#OB,OW<=G?[H,QRV`|IO'[M:$:Y!X@EK!<-O_DJ}&AH%V=[$BGOU?IVDIQ_Z&=D#(",
		"YF}MV*JDWUHR:UE,Z<;:$,;.)@P%CM&:CNWC!/]:%?-'_W}W@:AS,_=IN:(`?@~_R@.K:;Q.J_+C~{;NG:`':%[<GAW<~N'U:>Z=/A?KFN_];!OA/&.:%#?/KDE@`CO",
		"<{JOB/,OAL)FNGW:V>]X-'WSADF!.?^@Z-#PYCHUB?,TSNJN-)T'UP@*<VQX+N,{Y`|]Z:I:>:E+>JWZMIF&AQTHA=~!J#LGVB!ZN];'!@`;);*U;W^/~O~O!Y]/A|?",
		"=.|/)J?]YP$?*)*[&{;)Q<NFQ+,~:=>JF'SO|/P$EX>&M*=XU)N+$RA`_MJVYP$('V&[;/$'NXI&E#?SP=I~G(G.RU})?;:W!%/:SZI#}*)I:(Z.B`T-SU_J}GX#?[`",
		"=P+@;Q~!OL_,R<VX?U/`*^KY;?'Y^P=QBF-G^]<B@V^/$~|D$['U+<X[:?=LW_DPJR{&ZU*Q^'+ZB&T(R,>$W:D>F+[KBP)&|;$.UJ.;EKP@]!*UQF]VG,M[J&U'WBU",
		"FAMF<U.|M}DB*-U?=(NT@>G$'}XO@MYW<GK}UL<~.Y&,LTA=/:?)DSO`PCU*E;{P+LJ:&GF[<^#+<W?FUMEW,UMAF([G:ID~|_?PQ^L;(YPGJ_^YXUT<?)MVD#PEM+<",
		"P'/$RIB!?(DKI_WT(Z=['JUDBU}VWD&[*O{IQ^`VJ<,;',!-XVD~@,%WB.QSJXU*UH(Y^R:*$#FJ'P%&/B<D&CBF|O=FZ[^?PCU}H[}B`Z^<[X_Q~UN(UZ_P/#K+$!'",
		"`W&PV@/S+}>#)AODGU?*(;-!.:<=W+.QUK|+?E!,F.`J|LTV$}U'L%KS@R@^Y#VN]?R=>&PD+>LK_&RCT;=VCM%Q'%<,EDO%;)WCKG^OE.BJ^HNSDX.@Q&J^NY?^?MK",
		"VCT<^:)#P@)`Y(H>U.GQ,`:I(:|}<~O[P~KX<.<A(?]#X$)ZFCD.$)|^(.DCN[(S_)D_^M)*V&RX{LK`IEDGL&MF}/:BY*QMI_HVFW^F)S(?IK-.Y*NE@>=_{!UQ{_'",
		"%F?P}]KH]US}T[#&<;DXPG)Q@S>{#CAE+T+*M+;&ZIG<{KZGH%E.E]K/V]'%BD]>(%Z$PJ,NW:.[&YOBLSKD|H(<!_HOQ{-|:LC?U(Z|,)P<QI;E/GI`R~C<)J$}/AP",
		"GRQNP?^)A{ZH;{*=;(MYSA!.~^/(M+Z*'(XP.I/')+}>O?>Y{@`JKU&E_R^CR[+HGD)@E']!K*U()YJZ@UGD/K%.E?F(K#<S[!HAG@UY+}#'/LM-_A:{)PCSUTYR[S|",
		"ZBIKD~@C(MVXDPTCPDEX!DQMZUZY]TNT?=+*OS(OY#'|Z$P:+|.'Z>C-_>?)H%X]Z}O~>/DU=J[<+[V?Z_NRT)PG<-ZP=ZL,I[?L#AF.[O#R~$/U:_&L`FOY?E)G]=C",
		"&Z`ZA/$P'M.={*DQX+|F#C%:@FIZBK,C:?[#EN&.'=,I-YR`A@<ADZ:#+L<>|T,SO(TS?M[PV!):FIQYO/Z#]|WP&M*RD=D,/,F^T%F?UBZ*GM]-%NH+/'&Y:JXB:$/",
		"?U':GF]P.X{.]P@S^JBIX^RW=><#},!G%`~VNLCU!#XUO(._!MZ;>CL<T|Y_;*O.A(B(?C-@^(AM?-J'![$_^V}>@_@_S-S=$~#C>$A|:T#TO_>:,F[_G|%;:_C(>L*",
		"Q=&B#S<@+L!F(/K,J'|B!?_L%{QF}@|Y-B@BQ<_<]X!%/HW>[`AHCZX^/OVI!AX!#%Y;_';%|DWU=!?/>WDOZ_M_XO[D<J,VN.SFU*FR)LD}GK'^<X}AFQ'[V>)FLH`",
		"-LT~X-<M^),~I{=M?}&F/PE_U'<L@XK[QSIS.+ZCX%=TFSNJURG?.S`]J&WY${S;!TWTE>.;=Y@(S$K>Q+(TN(C@WOX,)&SHOGK~{/(B/<+P^PCR(N<T[,O-^~/;HJF",
		"C+/'Q[.;I*S@S$<^S'NL-DWR:![ZRQ#(<LN:P<=':;<#P$W<,=*)EOA.,I&]K{H_}A;_^H`JU!M)'-L:G=@/&W$OF|&'HSI@OLR:^:F',U[HR@&`UE(X/S)&*'-I=/Y",
		"^D_/LOS)BKDG=YF`_^K'(#YS#-SFZQ'U='~C]D[Q'^,YR.!@TZC*_-UVQP(<`R:<%@JW(FSU>;,M&Z#FM&%*V}SO!FI)'<HYTB^`%N=XC:DSU/SW[)]Q*@.DI!XG{&O",
		"=&!/,#T&%#IM:JD=[V}&%[RX,(~I<NA)@[=U%ZPI$-=V-.`N{.V<$.!@VDKXQDG}X#R~_H,>BGXSVF:&}.V{,~T,ON;_JDZ,AM%RI*QO]`?!O}:I/Y<}%,_C]CLI`-/",
		"(MVL-*,UE+&'.(WVN%DZU>|QG)B(MG|-,N%GM>L&_IH:%L|*+A>(FT>;#:|A@|)_B(*?*>IN^V?!E>`$X;LT+;:*DW-),G$/:M]'$Z-S!_*]&Y$ZP>A:*L=A/PU>:U)",
		":A/ILO+V+]`?N|^'[Z#LOW:$P]J:HWPZ_<U?X&BR[VFX$U=W(%[%~M*T^G;#OUEF&CV)=<W!;@+*(PS+T=_@X)/WK/(:(UDLT^>L]~P[_M?)=!}V&F/RZ&A?~WXQOW=",
		"~N-@E'KWOTX&>IOB,+V,Z?-#_IGO-HD_CWA~}E(<UOF,@#HI(RX%Q#*-WUP,(S':$OF,V*]I.I:YV:!.@-%:#<X=<UP,T$Q/JA([Z(M;&MZ{Z]PA_'|;*@~.,?-'GSX",
		"J+?W|BV%<!A=;UJ&`$U+]W}{I'%TNPV`S$Q];VINPHJ^;`+[ZLU'R&!_K/]DWT:V-IV#>RV&@^&;VI~IRV<XSCAF.ZKAB`;O:H];J/WO^&ACAC[*LS];KIZA`,<N}MO",
		"A`+]RC-!OU.?V%S/HKVIF&%AJ$~UB?]T)M{='N;I=O:=Y[Q>^P-V%ERMXJV=.]PD>)VRG@:PHE-MR}~,;>V*':M$F+)MK:J*>_^B_{%A+Z&_+L^*]X=&AL*TNBS<'}G",
		"&QZ[,B]%US<U%SX&Q+A![=GL#=*`<>WA+AJ<NA^J?J*;^X#Y/@KOMU,F,?K!_E</{PUKEVQXULOVZ%@&|!<^ALQ<HS+~|[DPHKHLZGF-DW*FNPCEB<SQ~YT;W,`Y(*`",
		"AH'+JU&(}YOI>THYFT:HB=`R<?FH$JP?DQ@<+FR#QMU$%BJLEW_$#>|@P/[B!#*S&;?U%FD(.G+KB:H+NX,P{^-&SZY+&|_=]S^#P.LYEX%_D]BW^+W.B*;ZUSCSCD+",
		"RWU)F@XD-MT>ONO)DYJG{,;.%G~?@+$YF[_^>]:YE-{.<T/?N-RBH<,+I*^H>I*'F'B&>D:@$-RLH]SAL$}M)=]I}&?PDN%P>/,W(T&MNSD%C/#P_~FCJ`Q]-A'K}SB",
		"Q^_J?JWGB;.%{YTK~%[E,O=<JT`@N.HVG/U&S:@P&%!V>%L`MD$RFX(OWT[UT.SPYL(S+E&X$LJ]F]KMA(<R+=*$K[.`?)S]=;B*|X.+Z$`<-%CO}X@MU~#)G@R#@)%",
		"LW)*#X)Z=RZ%B(SX?K$E_S,H`[>P,+AZ_P_|!NE[+VE:$+#PR.,]_]S+>/!CDEHOXGP=!*,S.YG%JFEY}DUOD_[/CV:W[*=HL',ULP/WR;A^STDR*[SQ[)OJ_`Y*|`;",
		"(>PZQ#-/.R%^O#EL%G(/D#&T[,QA.O.]_|IWV}?B_;)Z`Y*C<GOZ@VM/'Q|V;&K{&<VW+C[|MF/.=KOC@(P_RXMB)C-L:I&J_T$H('TSZEFRWI<J`.^!WXE%-(>/D_'",
		"&{]N!`BV`JXN_',*JUQ,+:>M>/VQ{W@#$(X%Y)Q_;LN`?I~FWR~+-/GQ@$D^MJ<@PKRJNDL}~AC,O<;D@_$GMS/)>$N(>^V=IZ~]=J%M-E>*<CN>Z/Q>E=&XT*+QF*,",
		"@NV)J/#@>T(O+(_ST%~/)~:)@#HM)(F:~MOWNSU/R&=@_BZ;B[{DTMNUO^?VE*!^);|]S_D$.LO,R+)R$K~YHQ}?],A/+PY[G+?UMN>{D*OU?$*}^`!$/@RFEQU-QKU",
		"<M!Z*BH[#_>)+M[T_:OU#V.~VZTGO{X,WF&O_T'_C!B^'?A;X.]}>%O}NZHV[T/J}[P+MK>]VCOZ-M>{^S/]>[!/GK^`$<-Z[O'DR^YRW_=:M[DT?K-&',JI?M%V/'D",
		"@+}%:=WE#JDZU)?ZG'QS,)@QF.P!~WV/.Y-W#H@A#GT;THVUA(VO^Y@(YKW%S)ALE<C!D=ZDX]RI`VYUM`*O+F/;[S)FG%`+;_O-_($L=?-MG*;X;NBS#M;=G&A&ER^",
		"!A~VRN)J@(@?OE$+.COB*'J@]PRGP^XI@$_I|@<MPMBYQ>.R%BU;EF~X#^;(V]>E_ZKZ|.S#Y&R%+AE'CYE%/Y{-WD/TO(Z/(=J$F>Z{:BFV[(!J(OQY=-S]ZTHV^+P",
		")GFJGRBFMJ/<D+'`#&U:}^'WK)%{S+=)/ESVC$+Z:*A/B$@I!{:,&/}GYQZJ]U=S(^VHZ,F[BG_~.A=*-.&JX.)J?[W}[FS#?,[L{QT'$VXM=$)I;BQDTV_MJIQM)DU",
		"K-N@>A%[N_?,W.][~<MVREO.]D:>XIJA(ZC.JE[`ZWQV$>RL.+=AS#/D#ANGI<)_SQ+L=GFH^DH{YQ&)]WI>QRY_`(C(S:%H&*P<`I.E*&.V&_W]!Y](H,{PZ^VCX$%",
		"]-&K%=?%-X.RHD#>=*$*S'.K@P~<TM@$V'A/O)MXS!(@VAU)-MP;ARIV@]TO=-S;L;O[!.'NY=PHR%M!T;}JA:BGHI<CX.$OU<*'H>RQ-Y&,LTAN[@QE;P&C-Z*.!:L",
		")MVCN(^],.%D@PYC(+D(DTD,'MOR/UN$BAI=Q`@?HJ}X(KA{O|Q#'E}&;%Y;T|J}Y,`L}RVCK&UP*MBN`T(K<Y;WSIMX?]TM%VUK#]`NPOB]=EX(,TE&XZGZRX^]YHF",
		"T%>,}<VCH|`=!,[S%HF>V=}C_TL',MVCZ&EX%IH`DMQ'AY@S{JP}AGB(]RMYEU}VTR?=N*JOK,G@-|^J^&NO$<IS;W+E}>+U$:*-@L`+CF[)CO+'X%I:,L@-,JD^/,*",
		".T/|.?Y?LJ-`N.+A$?JH}W#/T(#$>J}WCB^,=%;E'QF#!TV(VZ#H%;I^&@<;SU?AY[BHM}XOW;*H[#F?R/Z|XB,L^[>[&]K=N^>(:YOKMVTX(;J!^T;*UK?S^=B^A.N",
		"A-$[K?JG<*T>LYMJ*D;|*AJA<'MZP`~O)P[Y`/<@%U=E[~GBEJ.]ZUICEW`P?A=B}<I;`,)=?!/[;&{US/R.G-@JUE;M~YN$!)|]?RKUZ(*<{[F&'#&S#_<=<ME.E}S",
		"[!PRVX$G*_~EL%,>{(B|FV_LCO[DT.O-?|`~$_RY'JWC!Q;PH^?UHK/HT#YAUGAHQ-R^&#A}AG?UN._;J~=>Z#;J@O[/{O-OUK?%~O.K<C/_#DBGO=]._<EZ`SF.?WA",
		"<|DL:B+.RO,Y]V>ZE#V=YG-;`&SN>*RE+:?*<`W&N%M`.F|K^O)S,L@PG%!)A%<{$~`/B,;Z=)KHK[H]}|#.N>+E*U+YK%.#`#>/LKZO><V`NZSBURY[-.A+;^=XCS>",
		"FJT?R(I?{EW:].D'P]ABK#VAM%M!Y]O&Q|-M+UF[L][P~>.BZUNMAS-?J]?[TULG@K~[`!.}-!NV)#TSIYT:&N/P)<P-?J([RJ<*J%G}?->)'IQ)&WU/.<>QE#XJIVQ",
		"#}~`P%:[,X%TB@./JW=U&.P$U>+/RN_N}F|/RN-#(>PE&YW]#![Z<D{V_^!M&=-[QI$E<{)X&X`IS%M`%F`QK}XDW!%.-<B?|L#_U#B#H>T<SJPEPI#V>N;^QOX'KV*",
		"Y+._`+KAGVZV(ZX'&.$RKFX~'?-V&![R{<C:RD+{<=S'[_{(`,I:F+SK$<B]PTH^PN!R/_@)ZG>%Q]<OGUHD>A(#I{I/QO'&:#L*J^NUC.[?_CNDO,Z]DIMB*M?`KVY",
		")@L*A#!HZB}J)-WVHJ#_Q-;-/-/QG`~#KFD]:YO+<T'NWA%O,^C@%BAN(_GXIR[',U)Y(QKMJ*HFNPVM@XO!`):JG!,D[VTU%?FN^-B[,U[;P}&H&ZIPIO!$L?C-,'.",
		"Y%H+#O@:+=B@HFJ)%-[/IX/(@:$C>WZCE(>X,T/-=%AD|UVDA$I+T+,*DPH'_#%~U`OF%+(SX)*LN{&TX;D%W/R^+]&(Q=OB`%SUM?T`<JA:>S;+@?(+WF,^*-=~`+`",
		"|T<Q-$ENO:_`<?ABGN^GI$BF]:M>-&R]EM'Q>M:/;PD./VG~$&BW/PX[_TK-I~CX|])[&QLH?_`^IL*;`G+|'H@.O:]Q:{>'`R%(GN{#*,MO}$!V=+_B`./;,P$X:'V",
		"='-S}+{$|B.USA})S*VGS}`{/FI(`S^JQ|/WUFSAY_P+ZR?;KD$DFTZVEZ{>=(W':.PQ(Z:GY*<JN`NM?(W'RIZ=^(]{[PYOB_$H./M+P?K#AZ/~]>[/R,#AE{QD^MF",
		"=X@#T|':CU^?'A|!*FBFH(*-]`.~[GK'R&<ZXDV;F.ZQJH-I>S@$YUVB[!UZ.K,<];(TG-+_E)<]V&X=$*OB'N^$R^<[PLTDW?SY_OHZ,]!^Z,T:^[)=E_@WQOKY'#~",
		"=]Q$HW,NH.@T?%P:<XEG#:B.#UVH>O@V{[AQL`=EM<%X|;[$YFE[-N?/W[L[OIEHN=H]/J@%GP*$`-WHV(UP:)(KPDY:)>O+ODXVSZ}+>EZ/&ZV(AC_MI]OT%MBV(#L",
		"?_&L*(L[%E@T[/YC)D$#(!S?CWZ-P-ENXMJ[L+NO=Q.F+!F>LMC>BSR=<=Y*$*[$.B:~=!%JZ#U%LA@^@B-JE@I]TIVK)F-.N)NH-')/}EMA<R:FYB/Z/RA?L%L+T'C",
		"[+FBGS(G#;DI=ZM$RWXNY?#$+[,.?'W=;Y*][YP}]H-./B=DJI/$O!Z~<)>,~[$!`H;VI>K@#N@]?H!]T~UJ#UY|*><:FYHBED}L];<T+@].S,L]VW]MP[H^J$G[~)|",
		"TL`_)Q!}<)GBR`VZ<,{#E>GF<=NE?@~ML`OYP*!<(-]E${@NY(']_-Z&A.LIT*)G?I$[+Y&Z_OVMUI?[?&P{$WB,Q&!]U^ZL*$&(+#?%UDQZ*C!Q*)R!BA)#^IEK()O",
		"DJNTAT+A!.NOGB!<*K-Q%D:#-)C-M?_IAJ,=XY^+N#EW<@UN:VESOB{A-A*YEFV`.(!?UT^*ENA|J_<+;E`'QMCW'#/JOH]MBZG|X^IORT&,B({PR;RGBE?;#{^P(%I",
		"}!O~{+C`(P/FXTBV?J]NBNV&R/<RV?<!BXH?#`R=~<RN%]N{!@>!@_Y=,<I=`DT`NSA_[?]?T{@/K=(Q#*@%I}W=I+@I:|$C{E<HV&=*Z;`^QB=ZW]E.!RV?N[C]L>#",
		"PS)X_IP?WPDAT.{WIB('*EY,=K;('!GU/^MB*HM<>FAQ*]IN[`W*>QLJMB:NU&GFEQ>)$VWSY+OJ}~@$ZFQ&BOLMJ`JITLK@L!$B'U=IK-G(=UG<G/LX;ZH/W=!USB-",
		"I{|SA:PAZ#OJ?(@P'[.#W}Q.]A@CJH:Z@MB?$A<R{>#HSELZ%/N^C',Z(W/Y]O~E:/KWE;):*@?;?M^/J^_>ZF_Q%'@GAS-~I@:FP?Q|KDW>C*C%@;]*CL^/|CL[`~W",
		"=_:E*T$S<;Q[|XSXJD[]{<?N?Z?Y*W+;NCP?F)T}RW!:*P+;QFMO?%GKSE&P{@;IXZ@LD_YL*U^_C>IT&?<TYSGHNF.:!&P%:]VQ<&L@M(,;U<MQ!L%=!VS{CY'Q(D>",
		",%&J[H,JW)Q)!{ZX>{[Y`;)!KR[J^;+%!.&#=EZE(P$LUB`S]&PZ<_O)ED;Q:|&I<%&W'P-EYS?D$,VG:KEPW'=P>!+UEDBTN+I#-Y!D&WJ}N@=I;SH]CIFM-H}<+*,",
		"T/|-LU^?C.&PD'L~[CKR}K*DH$Z#FDPH(>?J?{WN#:G><~{!@E(VS,O`S]=<)Y>B;=_V>R-.!N}]R`U]OMUC*_M!~.~CG!,$CLI'L%*+~*!GDC+]>.MQ+IS@'&=%I$T",
		"IK'S=U[Q|)=HU,)A)G`?*%O=)V}B*A_RSO]TWS[VW_[$E)YKILJX)GWOH@C:O=%TQAC@T|-.@TF~)Y^C{R;B'GML<{HZ<+Q*!L]'JQWKVF!F?B%IR-,:L|O;CZC;^BW",
		"OUJCV>[&H#+-FYO-(>G~Q]I;T#L(:D'EPL:]F~RKWNZV-,NDTC)WIM)>(U_?:#_COFQ[@HGWF_PF]}`Z[D>!]*HAPDQ:^G`=?RN:G*AE&U}_BXFJ>CD%{]N)RML!$YJ",
		"<D]XPK=S`.}B[{S%@!R(!U`J,)$QT|B!S]:W}(]=]^BKJF_}OQMT@RH|+EZ/R./XT[?NYAFB-.MJT?|,+:|.)GJ;P*$DV+~TU&QY^Q<C/#[,{U|',U}JDTOQZ%K,@)F",
		"?-AUXR`;^%NKDH|#&LHW#=W`EUOVG@J-X)^P[}{J,QI*LYN#.=)R#OZEWML[>H</%ZX*B~N<#D$+PL*OQP_FT,;YD_]NIP(LEV!$-E+]ZXI},${Z{A:A:JB+;`.,+FQ",
		".UFSH,`D!N'A#[WO$YN@$/T+V`&:P#VXG,I&-V(LY%IDO'$'WF[SWC$JK)V>N>GJ#JTP#@%|(=.ZK)L!-IWG#:&O@XVL]S(Q_UG&#TUIV+Z.HN!}[^J_QEJB?[.K+=L",
		"<GDE=T+@-TY;)FHSG^<TX*(/[$&>~!DX;L:;EK`+:S|CXHFL)B~WT>R^'*^F_;]P[>H=%<-U*B*GU;N)[X(Q[#<K.[C{]UT>WM.K:#G]KFP)^)<D+W#`GYCB%,(LV,X",
		"@H&!HP>`HRJ?>`C#*.*MQ>YXAZ!^;A(':'EX@O{BP{U@IU$UXSR_~X@&W@VX,FI%S$F/D]N=>LI:'G]/N#!I+/_C)GPJ.[GZV#WO;!^;IA[X~J.;}Y$K.^U&IA?_=FW",
		"BC)Z-SWLY#L,BL@Q'KP,(A}=?NI#JZP&A[GQC^/~YK,^Q+_P`UG?=!{#[C{|!|O;-_$<J-/YEN(V,!~RN^]_&>U_YPD[K*`K;WQR`JDCU-IZ}PDWOMJG,<GX(SQD>AK",
		"@J$+)|FBNL%KM_;A$Q[Y@^V%.+M,Y]+ZD$-@'`T)<A/N;RNF,%!H`(?'#K#WT>%^GN_:;.S`~GQM;|O)TEMI.-|H:CGCB%S*['X}D'+(O_P'%@SGR%^IDS+|/X>V_A~",
		"+R&LAMN-<X*:UR.SE[!?Z,+^-R_X@J!W.`V.)M'NZ*|M,.]<M(VY}:P'H{/=<+M?&JRUJ,QR{+E(MJ'&W]*RTI$DMN};){R*~@#)ZPYH,&(XS-).*E{[&ZY`][SP>[V",
		"XWVCP-`F-X<_?(]J)O?'GU]FYN|O^.[K)FZ~XG(EJ':/LR.Y'P:F,^<H]M-SG=U.&W}~Q?P':P?`_YG]!L=RYB=ON<%BX$`S`G*V[(#_@.[+!/LXHG)(]|S|F:/NBWN",
		"GIAXVLMT`>IS/O@P')$VIM=X_%P.S/,-^U:&YB(]QE,|&-&:X*H+TE<WPHAI|=M@_ES%=EB*M[?<![>NVBL.+'O(P,@MCL}<[>+LSV!A#*U=HVUT)KSG`NEV!;U}I-!",
		"F$FVYA(AIC;^ROP}[A[.%K)(]D?E:%O$YLR.EXWP/LB/-?:<KCL/Z^<:/#_Q{>ED(T)T{YQZJMX@?:W)TLK:<CDZB#LHJ&#D<&O@X'Z/SIH&|=XJ=H=)~$?;KX~/&PC",
		"_Q/C):>/'~NXMJ'.P,?;N[L@N{J!T!K?]XN{<^<;G%HD![AJ[,NA(&C)}),DC&AI-(F><$ND#Y#_<=/X)YGN'?^SU.*&F!;PZD!EA#}N_-F-_:$U-IMG|#Z+RPN&/N*",
		"R'=!GYDB*]E^*F'H'^:GBP<H(VA?Y>Z:EC]A/U>(+WNJBZ!}M-'F>K$VG&{QE-*N*A[Q#;WH#K)X[A[=-~F/RS+Y`G@%;Q}.Y,J!~':<_N'`?.>#:+}<FUG?)`TEA=K",
		"<%O#(;(-ARPBKC~SD/JI/L@<KZ]M#~=T&B?G)C>W^.@+XD`$?;XB*]%'N%/@+!)OXLK!%;>_T#CX*O,Q|Q|^_=%_SG}G#{[MU@^*%$(NT?.O[UQ}];ZP{%=J<F?.'E{",
		"[G$)D/VDA_F(*|SG^ORWANO?)_>ZX+_)%&E!&E<,D:]VWRJ$]D-WHDI).[D+[DMBFURFIYQC)?=,VMWQFQZKFKME,SJI_<Q$C#WPHTVE;+#;.%*MI:LWNV[B>O#/=G:",
		"YXT#]E`>]%T$]S~>M?/.|^+>VHL&N)PF[PA)ADU}C[~ZVP`PAC-*':/%TE$LJ)BM/HION$]SU(IA~^EC,KJAK_:L`X&T-F-UG`-VNYGQPC$)P}`FP=KI.:~ALNG{()M",
		"XKQXYC*[!WL|%;{`J/HF.*Q^{MS?AO|=@|BZ,%;I!B{U>R<TS)X@*L}DC(DKJZ:?KW,&=?U(>.<]BED@$I,FU;I/;@ENAIQ^;$[{&%BK&LZR~}!YM<GD=}@QLW]LKHL",
		"/#/<-NU.LN`V~-=&>+%(UL~[+HK-;:#-X=.>R%Y'P@GY%{?KYE[#QO?%E]PMQ%'=!F}.D<=%<{XJ*]JYEY`'!-X>BGW<#E`@=SPC!ZG].)O~NK{IWSC_M@S=E;=H;*?",
		"XJZVH?':SY(+U`.[EX)X(OTB/HG'N'?KG,K:'GDU']^YH.A<.LN^]Z&)'YJM|:SOM)%QS(JY&WR&/Z%]ZQEGMKH}'),U]ZB]?PSCN;|U#X]<-T#D@`}X,QJA>SCWK#'",
		"NFI_B<SMW}K_BG$'TU-=S!<-}~.#)B;R'%ZY{_M-]L<VI[YQ|,G#RKCWU|KQ/UV@UP.T%+`_]Q:K]=#W?X.^.]L*&!ADF?*N/;Y(YF@_=(]DMH&HB%;GVZ]`,%K+FCN",
		"=~D$GCE|:^NQTJEDP!ZFDSR$U*L+D+^Y/VR{V[)-?=R.ZM!W(Q`_-QLCBK_W#_:JW:BIHICDOZ-A%TA/)#:]M:ZRO|CAIPFMS><.>JWMH^!SRLQ*?BG$-IYJY>X#O@/",
		"(ZX,%'R(I(/U^H#(ZV<]$D$)[)I.J~!>QB`V*X</WL'+<M<@)L(LTYN/,;U-|[#(UDRSJ&D;D:@PTV&`S`.RPU,$'P[XS$INCD<?BLVK}XO#CZ?.,TK.WYH;|*A*,:'",
		"XKI^:SKX[J>'?Y<_HJAS.!HZL'MB^_,;ET@NRTZ[Z/XK$MDS/;-UZ,DEK-:$'#Y`FVW/+*#X^I?N%J.:={.PKB.<*_Y>#!(A&/N-'L:VQ.BDQ{$[O%(LJ]K^|EV^)O;",
		"P*TWT$[U$'OP:W#_Q'_R`*O#~?&VO%(R)TJ$&P-BD=/?S*$RHDX^MI.R+$=B-'JV/>BFOT@-#^Z;WLAJQEFW'~S`KHWO.;C^DUSTYFEBD@<!(,]SOYQW^TWP>.$^S?$",
		"K?J%S]%}JUQ|TKHIXL>B&VX!>TG%ZOKR#GY+*=;^,Q#-H@A#<-&WTO}IQCR@*[#+G.NM$E)_!NQ<M[EAP.F/G'~/=)!V#<-PR,O{:CUR>DK+L):^ZE]DQY^NL(,PKVP",
		"S>HIXES&M(}CM[}P_#IPUK&$(L+%,]*YCMK$[QU,FEA,J+V[Z#~[TB<*<;S={^B$QHNJ@SD:U-Z>}[FT(T&!{N`Q!LU/#`[IU'!?FOKIW}W-A@'C[_=')-.*-LSL{C%",
		"T[J`@;)PA+.|;TA`N:;[LW*@(]QYMVA`OMV{@;%U_<,RELY=+!V;`@$~(%WM-I*I,@<IE<;_GH'JSQ;H-+<SA!_TZ~S>HZ?=#H&+~UD#,C,BK~H}`RM,]=/@KI~;<F!",
		"_RN=RUPRD]#RB:(EBUOU?[(YZ,~:]*[]B)M+<{_ZW#%FO;>Z:'^UZN,OU*{`_ZU%)/WM,=HP!PTO.D<BDGPZH&V`M!HE~`>%[:P'GFP@/M?!J=`ST_T)*%>BJL}PIE~",
		"@Y@F!QR$Y[T'*~S(K<GZ(PN!KYR[A}@O@+B@LC[#}PZQNS!/B+.{NABMWPDLB[$DV+~:[*HWI!Z'MLV#M]IC.L-#,FX^}C}`I.~=DCT+VQS~L|UP]-TP$W;F,_J|Y+B",
		"]AS*XC.K$|J{HP)JT[?L-V%O{|MFS[(CPX*Z,]/[S]LCGPUY:QVU).&]'P)?/C^_RDI:V|Y?}(KY(NJ[.BIA'V[(YE;NTFI(RJO<EZ:ZQ,HX<N!<F~'Q%XBHL=_UJ@=",
		"?@([Q#SFA-)S'FJ.%L,DT{;ZMK_L=&)S'C-}:R}A+P':E~-W_~{LG$J}$`@`-$_D=I%.=HS]#B?=L%{C_EDGOWDI&HEAK]RYT&OE.Q)UHEVMK|VNXM*#|I].Q_<A`/@",
		"{R(T{=`$+*YR#ELYG#O!MTX,Y'K}?IJXVK(O?({!/JD'P:D+NX=W'}#:RKINH^S'W%/IJ#,+}JP]/}R@`K/@#Y^*()SDO>=LMJF$[<*GTW;IQ)V[$^/AR_AQ]@SJ,W)",
		"F&$=CN<-<J|I#/]Y-;)WV)ED*;(.-$';*Y%^P?WTV(,/+</?Y-!+OZYQ@:($.ER=?RNR-#RA-HL&L)/&P(H(;J/M`#*`/Q[?%W':[#A-X;:=H%[:D@GE<RM${)@=OC&",
		";RLOQFOVE*'[IDWQ/;@*[.P(KX=#$XH}D?W(IXGX/>_NGB*T-LKW?'B'&HENO(CN&Q'~MAD<N@IGN!YS<!-/_^W=!-[@V`[QE$(=I~[*`>'T>MJ^>E_+VR.OTDJ])T=",
		"MCQ',I[DCSN!I/|RJ;JS-]<{!O[C;I,JA[X@YER;*:QYF%?A}QEI!@AMNV_:+K.*)S<;/BG'M)(H!];ON$X_D(D?ZVB<ZCXJKG-[;GBNH?_BW]L%`_?T^BQ&+%?TF(L",
		"C*'HZ()-V:G)@K:'^*&I-OY@C(LAL@J<HQ>`V(D|W;EDTKFBP=_X[ELVSEPU)SU}OW%UAQ:+DOL^$L[&/[!-+WU>AL(K]%^P[-~;=~V/]._X]TDS<K@FDP&ORNO>?]U",
		"(}*U&M)AWS?&=#N*^XUP(W@X]/EQ{@(VT-SIVRO+(*I@ZF~T?L@PQ$(F|@%Q>FAZF-%L>B%?~BG#DB]`C%|+[Y}>P_:`K%V@F}A)#]'K&<P>YS]G*M-XMH)QU=+ZLW#",
		":OB?*>!*]I;LA<ADQF&W[GQ&FGAO?%(P>'J[B=G*-ZR<R<ZV[^*J;P$N/DPK`U<:.'L'IHS{LZ`U.I|{YF=@AFGLEW*;>:~L}MK^U,L;.!X%;YBV-&X&@#S&`)BS&:@",
		"ZKIKX@L~_?LR'/?Q.T}I]GP@O_[`+()_I$YDAOVY<'=`MZCJ(YQK^TLS~GZCB^F[:$N[B->PTER$#N)!K^C'FL/BP>N=C]%LI>E'^WL*>[;_,H,$V=<T?Y![QK^DP($",
		"DB'LVZK%S%'$#?TMH|^%VI&'R]^|(_<,NAQCPTV_{O[@EY/`<K%'UIT[>FU(?|^X$:'DSC*ZGY'OPL?&T@DX=DA]V;Y]?N@NB(T&EP%:CA^-VART'/TA;N.ZA|^Z@~S",
		"{ECN:Q)VO,+=;%OHQD$N#)=L,-LR]T).'NRI+=[`+});=O*)[!H,`~MIC%U}!C})VS.A]JA&^%RY>B.`VG`V^ZVFRSKLD)+U`YQD!I^CD{U#U;*Z!Q`@`,X=,Z<QG#J",
		"$:.[{&]`|[-URJOC!F(+Z./U$FL:QR/}LID*^TWI*?W[/<Z/-@N.M#`@/-WKIY?V[)V{>+YBD/B;O]YP)CAES;NBUGJ_V$B[-Z]NL{WUX)VHY_SYD_UZ)YQRO&)>I^!",
		"DYASGMG&-%|F&`QAM>+`+HXHF<&|NM$Q*.XO:;X%|^QJB!^}G<W(E_/J=/;,L~`>)YS_`_A(TQ%)(E[D?&;/MXGMH.J$<X!I)-MFP(VU-!B*UV#JING~AGHMDR/Y}!Z",
		"!T&:}A#EZC(=SD{:U{(TOBM$G_=E;?&{G,{^/[J'}`@PJM*&D)[OT#PF_<^!GC:BXY#V+_#*ON'WDUG+X(,A-CF+:A#],M.S~CF]XH=$+FPHZ_G=GK}WO[)(>E&(/D`",
		"?![P&;B)@WTP:[J%VEDVF+&VAR[>^VUH@%?RCD.S-:_CT/A%$];C=I];B.G[V/{PH+=LUR%$MBEL=WK~IGWVTSA^J'+^;{HTOQ:{XGZH]/N#Q@J/,SFY(I$<OZ#DF|=",
		"@WN%LC|;$VCL_FUQJ?$?BHB.@HL&}=ASW*C)FH?LDK?VPFQ.>F<#+&,M.JT[FOS%.Z.]GK?!#=.&[OT#~{|GW+?)R:RL~T%V{D<W*F<IUF{+;]}CAC=DB,('OZ$Z@>K",
		"}%{$ZP)QFP-/N)FV!-_E[:P&=|L%Y';`>QLOYI?:-*!.L@.ZUFU/ANDUS>(G#(_O[DLM^K%SRAPIWZB>^.;W-X/?>EY+!Z^T;-FH[CX#ESZ?<`=RK*!B#TVZLWM&K#O",
		"U?_;HE<%QO'&~Q(G%S}X,T:*CD:#>SJ_X/^*.[~Y}~OG-&TQ?WEVXOBI*>B`=BPZV[QRCSF;V]D^(#SEDN|,*{,W'XKLZI:^S~`B_?+P^L]>PD,A*?FEN?Y+*=(Z/E`",
		"[<=>-,/D.@]TUI}_'[:UW~.}E-]}E-~FD$S$K.XKH&;:?H:DB]|%!C~SY#UJZY;Q:USTVIAURP(N{#_A$ZNU/!ER/W^*CK-G/'-Z'D)PDNT`<=J.)_CNO_ENW-@:!U>",
		":EW'IT%XS[E#)=C@:&J,)I-<_]?$I@X`^-YUD-Y>BY<[?!N{![=@DH#&D[UIMYCNYWSH+HB%!L#;,T-)G_{}$RIK,T`T)GCR&AC:`^+%KOCF'/MWR;[*}&NR/(,S]$B",
		"DB)Z-,Z#O+TXSD:N)F]Z;AMY|@&*PUSJ_TRBSF=$(XH^L`[J;`*?>#^`<#}XPO[Y_/A(V'X(I+XZ*=Z>IASXIVC#<GRM$GX-}J=B|$S%:+GV`GZ:VFY(Y@'KCB^!Y{J",
		"PZTU*;{D+A%H-H&H^#WC'AKM?:|E}C@HL+)^!XVN(TLM*I-?R#W]TD]#$#DT_KG;ZDW#:OV:Y*H@Q.EC%Q&RT.>[*M]P!-]QIVJTC?R}?I(Y(+)#CD{|[PI+#-X~=.^",
		";W>-?H@;YVR'{?@Q*Z^F<UJ{;W$/.UCN&=<^G*}SVG&YUJ`PB]YB-^%U%$ZJ)O:;M}]~P_.XLIPMS&B.,QY~X)Q^(=EP(ZG/!-')-I:~JB$QY?:*FBU{FQ/RFA.!|~;",
		"@$Q,&FD%KYG(}Y<>#.U#@<E!*:T`P$WC_.;P~`Y}[A)}^FD<^C^_&Z^BR!PJ<FWU-QDO_H?Y.?%Z{HD{Q~%;>`-.(]QWX~JYFW`[;FW&V([BEVNR^G>P:OKCT$YQVH[",
		">N/<(YW{.BG+S_$D*,MOIXC'JDR'QD=^!$!|>B&M{KOR>PET<FG%I~YC_+JKYN.YJOD|&/W=!L,&]-)'W(#D/!.FD+]J+JVKQPOC^?ZI?S_$E,A&/>$#SQEL%KA%K/X",
		"+L|>G#;&:?X<-EXIG]_#^O(DNE~(OF*Q^VKDT|^=Z_:HA?QDU{[};SQ&F*MG^S]X(AQIS+HQ['|&'$>D+]}NQE=KDIW)P={ZH?ZRP$P|!<EXZ+{Q`#AU#RG[^I^@$XI",
		"TOS*>UX+F;-<NDXY-AY-CES?S?KPKG?/Y!W-DT?-RA&E_F+$NX[#P?P#>`SP`FV&JL~@!BF@WQY)`('_ZY@VH[%'@XL$!F(ZOE-?X=D*C}-]>:/H~)}%>K!I{DG^MY'",
		"QP#G)E.F+.,;G>X*`R{&-/$<J}EO=D_|W'R:NKT!|%^SUX[O*!}ER*D=<_=N=KA*O{ASR#KFD>%ZS^]UV)O[AQ)'Z|<P>CD,>MWMV^:J)!.D:Q*MSA<.&'?$:%.>'*W",
		"N=#OB>_]KV&PLG+PX.-^%O&W:-@.B[OUGE,X/;CYHRL^QRC&A@;BQME@_]WERD,K{I<[WH'@`KCF@$!.,U>$V<@C(]UG?!{`VYVD?!L<~{<?@QUXD;^#@!Y>YJG&OHQ",
		"_~+PYDG^UPQUD*JPZD][>G~Y&-Z#S,SCO(X*SA/^].[.P[U$S]A$'FX#FCT#_R;=>-K_Z;W~}M&^@(IKOT!OW(R^<GL:JSDAZA@<BD:$H$[C+ZCYM`TND~UA?DZI=-F",
		"X*.{*RWJ@:;TP@B#^/,|*A[?@_/%+C?;]L({/I:CIJQP{T<&@WRZO:IUO>#ILICZ~HOPR%G^;%VGRX+WP;,FT$~.-^S`WZB*ML[>E|XC-YRJ`VYFHV*`;%]_?+T}A`,",
		"<~VP_&*$#}CN(=>!],+*(Q]<WAH)IK-KPD/C<LCW/;Q.Q'UO<;!%I'+KCB(!>^<>+~%+@JED/LOC_!#!-`)@M)WB#V{+M/AM]CXDNC&UCZW<SGP>/^W^SJT:A#B=>G?",
		",#WNZ!HSXEF+SDF*X;PKCIRY@-L!|.+BY)-Z(HWJ,^=V;K&U=HYR<$Z$J]-M!?JCDR}DR%_[^S.VSH;LQ*WH[^.Q=_Q$SPM&NK@NK+TG#W&)#HDKH:,NFR/_.NXUPJZ",
		"R'_JTF[$%W:ALU$O|[}.#)LNCG.B'Z*L%;I<KGP&JY.PSKR,_?U),$GQUY'_(A&VG{A(H<|@S)FPGD+>GFT}|%DE=_BMXYPO<$[J%V'JRMD^,!E<YXPK^EYVJ%I@(GF",
		"R;_CI|V}=K^I@MP/=T$*<Q$,J|P:CJ^%F:W(ZW;KGR,[C-(&,F|~%&>(XBNE^=_(+.|S.*Q(L:M.FRWL)N?[<IKNXE'/J`{,C*]UD]NMS~^-?M.)J)>/N`RFN.EFD_U",
		"LEN*@;@Z])XFN)/A/S*C',K?|NWKPBS)$<LS|TPH)WS'=_:P,['$*?KU.HCE;JRT_,EGQGP`FY:J('T:PAWM]W?C/_SK=|.U&NDY:!]};[~&/L+)]U[)YPH:'RS'^WV",
		"LUFLBGJHR&B.H#!>`JLHL}CFXQ#}:XENVH+J]&'/TPVQ<XIT%NK!#L-GEL=HRZQTF('Q)`XL<TA;([YWF-[MXD{X^/&{@P`O#BY[*<@!%(>RPME/[T:YUTESV-LPUP]",
		"CS@/TH?<*+OG`PH!>V*N&}[FRF.}S-BG,)Z&M=Y]O>N<[&L>K`-FOX_NZHSBU,I$BAN%M-A().WB(VYV^`-GOBKOSP+^[<TL%X=%+P+]@-@W_EYQOM+T;QLV#VTGB^%",
		"S~K/=|[GEU>NUZSF*$XBTW*_I+}&DW-C];`!V=TH*QL$MI=R([Q}H#|~U-(PX%=!}!WBKFV_(_`%/QK%{.J&H(CZS:TOKT:+;PBJA-'LW&)&-@Y/L.N|A{#I)'O*GU|",
		"(J:M.Z[KELPOEK|[R_IN%}Y?@G=U,GH)=#.|/PD!T>Y.ALM#R%(!%HG')%{*+Y?EI^=?']'S_XW`WI;YX.QTL_&?S|V$UY+[]V($-.=_E:PXW'G+AG[S:_T(`[ZN,A]",
		"G%M)WD%:D>_,/?%(H(U>M+<(EVKZA%O#CS(K|.@T*]^EM:C.U*O.!(?=D^CSL-;>]NXMW(~[%R;*`ML.:@G:M:?E>UBPK+KYG.T>!N?E):,TK[;VBM/L)O]H(EUPH`,",
		"{'`{R>S#KJR:'^KOHJPND@G/S&UT~A.'!OE<X:Q$SQHENAP#],]G$.]RJEZ=#[YRFJQ`~DGNCWF$+TP$PS^D@?[+&:|][VF}H'.^EM;Z:Z^>O,`C+~{^QFEC?(+;F!*",
		"_HS'J={C[G!+P<PF)O?@WVC@Z_S=[R{@Z[G[X:XV%^CXLO}<+=IJT`TVKGP'!%TXY[)}K|*N>X-SLD:/TGC`#L<HO./YALS;^L;+<Q^M+`ER_>#)CUD^=]:E]D`J@:J",
		".^!R>T;P+DURDMEKJ*[%<OIS@=MW!E,W|ZA?^NH@<I?[@D{]{_')&^A[?+]/FLN('|_/{WF-QLW+(!/'F`/QUN#I;!(>O^Z!}$P+F,B?+DYF<=/WI^;W$!B'PSZNO@]",
		"=&</EA!Z?[H$'DVL$)R<;.NW-],?#:)'^<*E?-M:_(}UM'+E[<()B%+)UA{FLR&R~%.NW%^T*JN+|*RJ,'&FW|J~VAO?V$NYO%XV>~N@L`J^RO~*V-?$G*~/N]?^<,.",
		";-,<^K?:AEPN^AG=GRT&)_C']COW}B;EP'C%,K}X*%=._BJE&>CH`]$_I/~?%E;~NI`T?{N.(&B'DPA,$_JG_RI}E/{T]+*>{X_={J]BMW(&*W[)Z;QV+IR|F:B]~E'",
		"U~>*LBO|YGIZY,(>MCV=NG=E!`G-DV`K:]WD<YBJ_H-(-N@XR>{/#KZ<)L/JSAG-B.EY#/'LWR$|D}EAQUFS/;!.L!B@'{L>X*,.Q;<'<JS<H&Q$>*H&V:(/W_'{&*W",
		"D`]_EK:/;U[,ET.Z(&(|-IFQK$^X^Z?)+MIKNR[S#J<U.YN,&FWDEZ}?J*(!&#_.G.:Y+Y:|=ANKFUKO!RAP_^[$(!%MAWTXG~BY{Y^,KL@EY+W`;W!#([(LI&U^&`^",
		"Q;?!,F.`.,S{/R:!QORXKXPUQT,HBW][X]IY@&H!L[_#K#B:{|P<N[T`.VDXNW_UM]K.,*;$J=LJT+HX~Y>`:,%R?J`ICV(;T)^P;}$^L&D(V'&]AYL'|,W]RJKHDPV",
		",?),B`?`>.HQ&QY_D+)|-V^W<NIEYUR<'W>C[!D]Z[K;$_`FT!+(_.N`V>W/^I[_`Y&/K&AV-,(TK:ON+);X_/YA#A(O.,S:Q[&U:]L[;N(VH(>Y[E[GA{(MPKZRSAG",
		"$;*_K;.*}{XGVDFJ!B-GEGXA%?VUF#I*E]NBN-NELITBEJE+C+MOYU.N/}.^MA.JR;TR;^%/Y-V:?;LWGX=TCA^UQKL'F,+T~|/)N=L=$R%YE?AKU,Q}EA:]A?.^[RF",
		".,#/|~)+P>)RJ#B%,'P-$ZIYSDT[|K?O({:<|J?_YGW$>N].-N#^M{G*YQ*N>KX_TZ_]Y$:$&$JRW#|T{][`LUMGO+$_~'=|:@']*LQN)FD$-Y>B~=QU&#,`V@/?HU|",
		"/XVMUDQA=C<CRWGT`L;Z@J>GL;|RWVZOPB?.):Y*TNJQ#ZNWRDTCPTY.<GE]*,ZW&FHN/`)TUJ.B%^P^P@$!,T{BDFT)J.,]M&-FM)FHS#HA.WZ|>;-Z`&(ND_=#E<J",
		">-TKCFPDCAK%`%-V<P:)R^D{-{C`!A;LU](<$D#`<$MB]=?](]'M$`GC[>Z:A#>FHS[?YSEG;DW?%O!-JQ|]#(H^FU</P*QSQ_Z+FL$N:S=C)XR[@&)YL+VAY&E@TYK",
		"'#)/Z=VP[X~D+V{PA(%'J`QL]-&]Q)R^#F!A^EK`F_,Z%KR|Z!@IN;PSPK[I#)_NTB{CQF~C/U$+X[=D!YF)SZC-&O`A,FP`'_SXMY;MKZPDXOA}=C(CH?IO^<*%G?=",
		"A,E.;^J](AFW~X!OQV!H);EL#WZW?PG|(H_!IYQE@<R]X+RZ-_S)Z?U@IM-=]{GRM+?~T,O%:-LD'EZSW+^E'NX;|YR[<WU}^=%^YS`@&S)%:|NS@C?]P>T*.`M!<@*",
		"Y'YKF[U-Q$*P^FMT(H<YS?ZT(EH>RXCN/K-~O.G$'B`I]M:Y#J{,>/(?P$^-<CR:I~LN@YA@#H^K)<_H>&ED<&X'WA~&KCAE+$:)^'^UGWN>WTX,#FBUP!%C|S&I]O$",
		"BJU(#IFK(FO~M{&<K+#,IH@|;H]O.Y!K$(ABM-UA%`#B%/T};WA$*EJHV%FW)NS+@F^M;Q.VR_>D[*+*TD(M`M&/#D$%:G/D>]LMUNX=RVM:`NJ!,+JQXP=U|SE,ZR!",
		"/M]&=,WGUCHR*#J'_D<E^&)KATKG*MXDWJ%]HRG},[}AQ:L@DU()L*'.Y+P;RNQOYF%UR`Y?@'EPD>^+]JZQG;VBFVRJ^#$A[CG<._RNSKXM=#S%{G<F'Y'DPS}LIF_",
		"-.M['?V`P'$Q@OT,Y&R$'}%M&.U'W#ATN.P_!$G}QW%QKS^[PJXIX{'&M>$<(@$R{VLY(;*OE^AIW]X)KUN?H<PV#V@GR#WYVP<JDSO.D}FY)FE~ROLIHN|@I+N@ZVD",
		";@SO~F!N|X`D$L{%W^#;MRHR;HM;/.'JMR%K;:E=T.|^&!X_)ARH:'PRJ#Y?VI#YE{!EID$EV*TA[:F)<~G._D+F~VWP;#_F'=MLE$'UAUEM&>FT*:]>Z&AY*'_T[K@",
		"_QUF(&>?R:$Z}PT-/>_%@TKYUF,&_:<:RB%KW!:~/=)(H[:/P+T*FB_]>L:B-BL,H.<KVSENFR@*+CL~O*K/X=Q$.-`W_AV*[W>!D`MCKV)Z><+I)U+OENMEH-&K.'=",
		"GB;PXR;#@~N%{OW@A]J?LX{N^;DS`:Q_#QG,$Y^>;$Q<;UYICT/N~}EF'_^~$#T$<*P_-#`ES+MCN`FP`S?(}<FM?VH&#XKU='H|!JV(KU<`,K(X%OL|Q@B>DYFEC*U"};


void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udpClient_send(void);

extern struct udp_pcb *upcb;
int counter = 0;
int UDP_Init=0,start_UDP=0;
const char *message_init = "%&hqt6G+WuXa4oq*uISC?V20k{gpRgcE&#G_0A62rua7vEoc*2+JrZuHaW*ZSr!=LT=yVK)ef-)w5p[gjyI{emT4nk=C*%QKQ#[Tuk}HQ0){ISk#JYrxUJ";
const char* confirmation = "4jqz484yl94neddq0twxugnnyty6imjyc5zdeyyizl636mvk48pi1as8fnyc01a9lj3mamlp4jdcmjfviw48uv7fv4mv52gq75atzpus853ov2n8phy59cy3a77wp";
char message[keyLen]="";
char buffer[keyLen]="";
char message_cipher[keyLen]="";
int counter_UDP=0,len;
int RFID_Received = 0;
uint32_t board_serial_number;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;

void fill_dummy(int start) {
    for(int i=start;i<keyLen-1;i++) {message_cipher[i] = rand()%126;}
    message_cipher[keyLen-1]='\0';
}


void XORCipher(char* data, bool send, char type) {
    if(send == true){
        message_cipher[0] = type;
        for(int i=0;i<strlen(data);i++) {message_cipher[i+1]=data[i];}
        message_cipher[strlen(data)+1]='\0';
        fill_dummy(strlen(data)+2);
        for (int i=1;i<keyLen-1;i++) {
            message_cipher[i] = message_cipher[i] ^ key[counter_UDP][i-1];
        }
    }
    else {
    	for(int i=1;i<keyLen;i++) {message_cipher[i-1]=data[i];}
		for (int i=0;i<keyLen-1;i++) {
			message_cipher[i] = message_cipher[i] ^ key[counter_UDP][i];
			if(message_cipher[i]=='\0') {break;}
		}
    }

    if(counter_UDP == numkeys-1){counter_UDP = 0;}
    else {counter_UDP++;}
}


void udpClient_connect(void)
{
	err_t err;

	upcb = udp_new();

	// Bind the updb block to module's IP and port */
	ip_addr_t myIPaddr;
	//IP_ADDR4(&myIPaddr, A, B, C, D);
	printf("\n\rClient IP %d.%d.%d.%d\n\r", ipv4_address[0],ipv4_address[1],ipv4_address[2],ipv4_address[3]);
	IP_ADDR4(&myIPaddr, ipv4_address[0],ipv4_address[1],ipv4_address[2],ipv4_address[3]);
	udp_bind(upcb, &myIPaddr, 8);	// UDP Port 8

	ip_addr_t DestIPaddr;
	//IP_ADDR4(&DestIPaddr, A, B, C, D);
	IP_ADDR4(&DestIPaddr, server_address[0],server_address[1],server_address[2],server_address[3]);
	err = udp_connect(upcb, &DestIPaddr, 5005);		// UDP Port 5005

	if (err == ERR_OK) {
		message_type = 0;	// Initialize message
		while(UDP_Init <= 1){
			HAL_Delay(200);
			ethernetif_input(&gnetif);
			udpClient_send();
		}
		client_connected = 1;
	}
	printf("\n\rClient connected to the server\n\n\n\r");

}

void udpClient_send(void)
{
  memset(buffer, 0, keyLen);
  memset(message_cipher, 0, keyLen);
  struct pbuf *txBuf;
  int len;
  if(UDP_Init == 0) {
	  UDP_Init++;
  } else if(UDP_Init <= 2) {
	  message_type = 0;
	  board_serial_number = HAL_GetUIDw0();		// Get STM32 Serial Number
	  len = sprintf(message_cipher,"0%x%d%s",board_serial_number,room,message_init);
	  printf("\n\r[%d] Initializing...\r",counter++);
	  //printf("\n\r%s\n\r",message_cipher);
	  //printf("\n\rClient %d.%d.%d.%d trying to connect the server\n\r", ipv4_address[0],ipv4_address[1],ipv4_address[2],ipv4_address[3]);
	  //printf("\n\rGate:  %d.%d.%d.%d trying to connect the server\n\r", gateway_address[0],gateway_address[1],gateway_address[2],gateway_address[3]);
	  //printf("\n\rServer:  %d.%d.%d.%d trying to connect the server\n\r", server_address[0],server_address[1],server_address[2],server_address[3]);
  } else {
	  if (message_type == 1){	// If message contains UID information, send it encrypted
		  XORCipher(message,true,'1');
		  len = keyLen;
	  }
	  else if (message_type == 2) {		// If message contains BME information, send it not encrypted
		  strcpy(message_cipher,message);
		  len = strlen(message_cipher);
	  }
  }
  // Allocate pbuf from pool
  txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

  // Copy data to pbuf, send and clear the pbuf
  if (txBuf != NULL)
  {
    pbuf_take(txBuf, message_cipher, len);
    udp_send(upcb, txBuf);
    pbuf_free(txBuf);
  }

  if(message_type!=2 && UDP_Init <= 2) {
  		udp_recv(upcb, udp_receive_callback, NULL);
  }

}

void receiver() {
	if(UDP_Init > 2) {
		udp_recv(upcb, udp_receive_callback, NULL);
	}
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	// Copy the data from the pbuf to buffer
	strncpy (buffer, (char *)p->payload, p->len);

	// Manage receive messages from the server
	if (UDP_Init >= 3) {
		if (buffer[0] == '9') {		// First bit is 9: Emergency System
			printf("\n\rEmergency!\n\r");
			Open_Door(1,2);
		} else {	// All other options are the reply from UID message sent by the client
			RFID_Received = 1;	// Client received an acknowledge from the server
			XORCipher(buffer,false,'1');	// Decipher message
			if (!strcmp(message_cipher,confirmation)) {
				printf("\n\rUID accepted\r\n");
				Open_Door(0,0);		// Meaning it is entering the room (no emergency)
			} else {
				// In case of no access, the Red LED will turn on
				printf("\n\rUID not accepted\r\n");
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
				TIM5 -> CNT = 0;
				HAL_TIM_Base_Start_IT(&htim5);
			}
			printf("\r---------------END-OF-REPLY-MESSAGE---------------\n\n\n\r");
		}
	}

	// Initial synchronization
	if (strcmp(buffer,message_init) == 0) {
		UDP_Init++;
	}

	// Free receive pbuf
	pbuf_free(p);
}



