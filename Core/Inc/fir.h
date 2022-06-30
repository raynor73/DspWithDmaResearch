/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44100 Hz

* 0 Hz - 10 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -60.002310530464435 dB

* 200 Hz - 3000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 4.174803458680513 dB

* 3190 Hz - 22050 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -60.002310530464435 dB

*/

#define FILTER_TAP_NUM 343

static float32_t filter_taps[FILTER_TAP_NUM] = {
  -0.00025373409301121443,
  -0.0013456980322589808,
  -0.0017360529904533629,
  -0.0027881021408700073,
  -0.003878952400251391,
  -0.005061522085259879,
  -0.006181271147449174,
  -0.007101870657273964,
  -0.007666884902174353,
  -0.00773254420289289,
  -0.007185689985640183,
  -0.005964647070201558,
  -0.0040756339086721275,
  -0.0016008832502949397,
  0.0013012374587971058,
  0.004407682166933235,
  0.007451870523109172,
  0.010152335936262715,
  0.012247240397276534,
  0.013527332228985636,
  0.013865831900282614,
  0.013237985723573901,
  0.011728259832764041,
  0.009523478409730288,
  0.006890909793570129,
  0.00414514701225811,
  0.001607382431678897,
  -0.0004376209843780807,
  -0.0017795934882171031,
  -0.0023111042604777145,
  -0.00204162758140566,
  -0.0010940420961491264,
  0.0003148168535543788,
  0.0019077510508507675,
  0.0033903509665064264,
  0.004495530142971052,
  0.005024437728176947,
  0.004876518960593445,
  0.004063207118994387,
  0.002704387724298115,
  0.001007495356962268,
  -0.0007679173178801292,
  -0.002353252337222526,
  -0.003515299959719235,
  -0.004094943478226603,
  -0.004032673642947639,
  -0.003377477366658778,
  -0.0022777492635874946,
  -0.0009550534039564886,
  0.0003345123962247241,
  0.0013444649593989401,
  0.0018803172986232935,
  0.001833257809136597,
  0.00119854936799197,
  0.00007575076604603971,
  -0.0013489317413887792,
  -0.0028358153018496392,
  -0.004134544263069955,
  -0.0050283898266724626,
  -0.005372842552072532,
  -0.0051217856545890585,
  -0.0043359343346346425,
  -0.0031721642169413367,
  -0.0018556362644367573,
  -0.0006396504556789512,
  0.0002399871918721236,
  0.0006081004767355518,
  0.00038221410057019125,
  -0.0004119200169327613,
  -0.001643855975111515,
  -0.0031012946765433644,
  -0.004528383780984098,
  -0.005672105756730538,
  -0.006328643007311766,
  -0.0063814133160927,
  -0.005823694861131114,
  -0.004761436170282774,
  -0.003395155014448204,
  -0.0019843098143074163,
  -0.0008004838748495364,
  -0.00007736345360409878,
  0.000033340246183053485,
  -0.0005086153999912001,
  -0.0016227855264490249,
  -0.0031207177775812883,
  -0.00473885480265169,
  -0.006186156189631342,
  -0.007197650184372481,
  -0.00758420320612008,
  -0.007269399858794685,
  -0.0063061418501278565,
  -0.004869639491286676,
  -0.0032276639106091336,
  -0.0016926312270769534,
  -0.0005642538711693826,
  -0.00007323875671931173,
  -0.0003365347032795396,
  -0.0013327310687396995,
  -0.0029023913180589007,
  -0.00477414327956092,
  -0.006612520346757758,
  -0.008078939482628587,
  -0.008894993038496281,
  -0.008896442135094997,
  -0.008067197701569667,
  -0.00654667425053934,
  -0.004608391062843035,
  -0.0026127323166567616,
  -0.0009422069281415376,
  0.00006945402427110394,
  0.00020150496518598647,
  -0.0006085806085012883,
  -0.002248117546907246,
  -0.004444389243263461,
  -0.006810257831476003,
  -0.008912880351029866,
  -0.010353177279012762,
  -0.010841706262125016,
  -0.010257247370268526,
  -0.008676175066555474,
  -0.006365611584641144,
  -0.003740827023056201,
  -0.001293175544140583,
  0.0004990580832334502,
  0.0012605579161411487,
  0.0007939757552641777,
  -0.0008749262302707999,
  -0.0034922365982106556,
  -0.006613297924956894,
  -0.009677352165947865,
  -0.012106994028957253,
  -0.013415180196933706,
  -0.013301437864027234,
  -0.011718153110202405,
  -0.00889216188542258,
  -0.0052966860832413636,
  -0.0015766469738051475,
  0.001561798391053863,
  0.003480019007329307,
  0.0037309437610547505,
  0.0021534931167166245,
  -0.001079883677603831,
  -0.00547281868206734,
  -0.010275466838724991,
  -0.014607465054931074,
  -0.01761169484006632,
  -0.018613627794471885,
  -0.01726046087229842,
  -0.013612355023209554,
  -0.008163749609076685,
  -0.0017895748878379585,
  0.004378995500245442,
  0.009137971732829014,
  0.011421042213382523,
  0.010502223138726537,
  0.006162091690711275,
  -0.0012188710102918221,
  -0.01065990667954949,
  -0.02067421579553018,
  -0.02945453993536036,
  -0.03512620817873684,
  -0.03602557350973434,
  -0.030972006112096506,
  -0.019484004946200506,
  -0.0019039879894895285,
  0.020583893205221096,
  0.046062126272628776,
  0.07209261410440092,
  0.09601098600277974,
  0.11526733962888358,
  0.12775048911516368,
  0.13207389460674857,
  0.12775048911516368,
  0.11526733962888358,
  0.09601098600277974,
  0.07209261410440092,
  0.046062126272628776,
  0.020583893205221096,
  -0.0019039879894895285,
  -0.019484004946200506,
  -0.030972006112096506,
  -0.03602557350973434,
  -0.03512620817873684,
  -0.02945453993536036,
  -0.02067421579553018,
  -0.01065990667954949,
  -0.0012188710102918221,
  0.006162091690711275,
  0.010502223138726537,
  0.011421042213382523,
  0.009137971732829014,
  0.004378995500245442,
  -0.0017895748878379585,
  -0.008163749609076685,
  -0.013612355023209554,
  -0.01726046087229842,
  -0.018613627794471885,
  -0.01761169484006632,
  -0.014607465054931074,
  -0.010275466838724991,
  -0.00547281868206734,
  -0.001079883677603831,
  0.0021534931167166245,
  0.0037309437610547505,
  0.003480019007329307,
  0.001561798391053863,
  -0.0015766469738051475,
  -0.0052966860832413636,
  -0.00889216188542258,
  -0.011718153110202405,
  -0.013301437864027234,
  -0.013415180196933706,
  -0.012106994028957253,
  -0.009677352165947865,
  -0.006613297924956894,
  -0.0034922365982106556,
  -0.0008749262302707999,
  0.0007939757552641777,
  0.0012605579161411487,
  0.0004990580832334502,
  -0.001293175544140583,
  -0.003740827023056201,
  -0.006365611584641144,
  -0.008676175066555474,
  -0.010257247370268526,
  -0.010841706262125016,
  -0.010353177279012762,
  -0.008912880351029866,
  -0.006810257831476003,
  -0.004444389243263461,
  -0.002248117546907246,
  -0.0006085806085012883,
  0.00020150496518598647,
  0.00006945402427110394,
  -0.0009422069281415376,
  -0.0026127323166567616,
  -0.004608391062843035,
  -0.00654667425053934,
  -0.008067197701569667,
  -0.008896442135094997,
  -0.008894993038496281,
  -0.008078939482628587,
  -0.006612520346757758,
  -0.00477414327956092,
  -0.0029023913180589007,
  -0.0013327310687396995,
  -0.0003365347032795396,
  -0.00007323875671931173,
  -0.0005642538711693826,
  -0.0016926312270769534,
  -0.0032276639106091336,
  -0.004869639491286676,
  -0.0063061418501278565,
  -0.007269399858794685,
  -0.00758420320612008,
  -0.007197650184372481,
  -0.006186156189631342,
  -0.00473885480265169,
  -0.0031207177775812883,
  -0.0016227855264490249,
  -0.0005086153999912001,
  0.000033340246183053485,
  -0.00007736345360409878,
  -0.0008004838748495364,
  -0.0019843098143074163,
  -0.003395155014448204,
  -0.004761436170282774,
  -0.005823694861131114,
  -0.0063814133160927,
  -0.006328643007311766,
  -0.005672105756730538,
  -0.004528383780984098,
  -0.0031012946765433644,
  -0.001643855975111515,
  -0.0004119200169327613,
  0.00038221410057019125,
  0.0006081004767355518,
  0.0002399871918721236,
  -0.0006396504556789512,
  -0.0018556362644367573,
  -0.0031721642169413367,
  -0.0043359343346346425,
  -0.0051217856545890585,
  -0.005372842552072532,
  -0.0050283898266724626,
  -0.004134544263069955,
  -0.0028358153018496392,
  -0.0013489317413887792,
  0.00007575076604603971,
  0.00119854936799197,
  0.001833257809136597,
  0.0018803172986232935,
  0.0013444649593989401,
  0.0003345123962247241,
  -0.0009550534039564886,
  -0.0022777492635874946,
  -0.003377477366658778,
  -0.004032673642947639,
  -0.004094943478226603,
  -0.003515299959719235,
  -0.002353252337222526,
  -0.0007679173178801292,
  0.001007495356962268,
  0.002704387724298115,
  0.004063207118994387,
  0.004876518960593445,
  0.005024437728176947,
  0.004495530142971052,
  0.0033903509665064264,
  0.0019077510508507675,
  0.0003148168535543788,
  -0.0010940420961491264,
  -0.00204162758140566,
  -0.0023111042604777145,
  -0.0017795934882171031,
  -0.0004376209843780807,
  0.001607382431678897,
  0.00414514701225811,
  0.006890909793570129,
  0.009523478409730288,
  0.011728259832764041,
  0.013237985723573901,
  0.013865831900282614,
  0.013527332228985636,
  0.012247240397276534,
  0.010152335936262715,
  0.007451870523109172,
  0.004407682166933235,
  0.0013012374587971058,
  -0.0016008832502949397,
  -0.0040756339086721275,
  -0.005964647070201558,
  -0.007185689985640183,
  -0.00773254420289289,
  -0.007666884902174353,
  -0.007101870657273964,
  -0.006181271147449174,
  -0.005061522085259879,
  -0.003878952400251391,
  -0.0027881021408700073,
  -0.0017360529904533629,
  -0.0013456980322589808,
  -0.00025373409301121443
};
