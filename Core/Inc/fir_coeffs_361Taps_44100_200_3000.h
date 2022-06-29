#define HILBERT_AND_DELAY_FILTERS_TAP_NUM 361

static float32_t coeffs_hilbert_361Taps_44100_200_3000[] = {
  -0.0141508892,
   0.0018432517,
   0.0011465772,
   0.0002656717,
  -0.0006995893,
  -0.0015693929,
  -0.0022235882,
  -0.0025718063,
  -0.0025909626,
  -0.0022989373,
  -0.0017621006,
  -0.0010720145,
  -0.0003432160,
   0.0003099051,
   0.0007833769,
   0.0010061645,
   0.0009474820,
   0.0006256154,
   0.0000998708,
  -0.0005376049,
  -0.0011805651,
  -0.0017230476,
  -0.0020778532,
  -0.0021878813,
  -0.0020374502,
  -0.0016539740,
  -0.0011058786,
  -0.0004899964,
   0.0000838414,
   0.0005135269,
   0.0007217384,
   0.0006708376,
   0.0003687738,
  -0.0001308639,
  -0.0007385228,
  -0.0013432132,
  -0.0018330512,
  -0.0021151915,
  -0.0021343108,
  -0.0018830846,
  -0.0014049958,
  -0.0007863058,
  -0.0001410515,
   0.0004103776,
   0.0007630664,
   0.0008479550,
   0.0006451188,
   0.0001891703,
  -0.0004363553,
  -0.0011133004,
  -0.0017110673,
  -0.0021110153,
  -0.0022298559,
  -0.0020360640,
  -0.0015574380,
  -0.0008769082,
  -0.0001183996,
   0.0005761174,
   0.0010731664,
   0.0012741051,
   0.0011350372,
   0.0006772089,
  -0.0000155174,
  -0.0008110094,
  -0.0015534665,
  -0.0020927990,
  -0.0023141058,
  -0.0021607311,
  -0.0016471432,
  -0.0008580397,
   0.0000658210,
   0.0009535287,
   0.0016362611,
   0.0019804399,
   0.0019146260,
   0.0014453872,
   0.0006581181,
  -0.0002970889,
  -0.0012327999,
  -0.0019593372,
  -0.0023216814,
  -0.0022305529,
  -0.0016819966,
  -0.0007607284,
   0.0003734383,
   0.0015144815,
   0.0024487116,
   0.0029962213,
   0.0030468191,
   0.0025837822,
   0.0016900268,
   0.0005353649,
  -0.0006537423,
  -0.0016362665,
  -0.0022031496,
  -0.0022189865,
  -0.0016509290,
  -0.0005783862,
   0.0008186191,
   0.0022911943,
   0.0035675637,
   0.0044051939,
   0.0046388615,
   0.0042155399,
   0.0032085717,
   0.0018079362,
   0.0002873542,
  -0.0010459052,
  -0.0019096405,
  -0.0021023897,
  -0.0015455309,
  -0.0003029356,
   0.0014261605,
   0.0033409057,
   0.0050934195,
   0.0063546855,
   0.0068791889,
   0.0065559409,
   0.0054352480,
   0.0037253140,
   0.0017579744,
  -0.0000707794,
  -0.0013751565,
  -0.0018548269,
  -0.0013567867,
   0.0000888453,
   0.0022669111,
   0.0048111497,
   0.0072687808,
   0.0091847264,
   0.0101898786,
   0.0100767023,
   0.0088468922,
   0.0067208296,
   0.0041051695,
   0.0015230249,
  -0.0004817185,
  -0.0014486713,
  -0.0010943167,
   0.0006241013,
   0.0034877848,
   0.0070449649,
   0.0106857905,
   0.0137517121,
   0.0156601826,
   0.0160218376,
   0.0147273323,
   0.0119857288,
   0.0083042593,
   0.0044099514,
   0.0011243745,
  -0.0007876043,
  -0.0007682443,
   0.0014118780,
   0.0055864420,
   0.0111959579,
   0.0173645595,
   0.0230420647,
   0.0271897582,
   0.0289791773,
   0.0279686896,
   0.0242244283,
   0.0183597101,
   0.0114798323,
   0.0050347126,
   0.0005980183,
  -0.0003949260,
   0.0030908753,
   0.0114718762,
   0.0244105289,
   0.0407900049,
   0.0588100694,
   0.0761959773,
   0.0904938460,
   0.0994110577,
   0.1011516168,
   0.0946954066,
   0.0799774556,
   0.0579374939,
   0.0304292463,
   0.0000000000,
  -0.0304292463,
  -0.0579374939,
  -0.0799774556,
  -0.0946954066,
  -0.1011516168,
  -0.0994110577,
  -0.0904938460,
  -0.0761959773,
  -0.0588100694,
  -0.0407900049,
  -0.0244105289,
  -0.0114718762,
  -0.0030908753,
   0.0003949260,
  -0.0005980183,
  -0.0050347126,
  -0.0114798323,
  -0.0183597101,
  -0.0242244283,
  -0.0279686896,
  -0.0289791773,
  -0.0271897582,
  -0.0230420647,
  -0.0173645595,
  -0.0111959579,
  -0.0055864420,
  -0.0014118780,
   0.0007682443,
   0.0007876043,
  -0.0011243745,
  -0.0044099514,
  -0.0083042593,
  -0.0119857288,
  -0.0147273323,
  -0.0160218376,
  -0.0156601826,
  -0.0137517121,
  -0.0106857905,
  -0.0070449649,
  -0.0034877848,
  -0.0006241013,
   0.0010943167,
   0.0014486713,
   0.0004817185,
  -0.0015230249,
  -0.0041051695,
  -0.0067208296,
  -0.0088468922,
  -0.0100767023,
  -0.0101898786,
  -0.0091847264,
  -0.0072687808,
  -0.0048111497,
  -0.0022669111,
  -0.0000888453,
   0.0013567867,
   0.0018548269,
   0.0013751565,
   0.0000707794,
  -0.0017579744,
  -0.0037253140,
  -0.0054352480,
  -0.0065559409,
  -0.0068791889,
  -0.0063546855,
  -0.0050934195,
  -0.0033409057,
  -0.0014261605,
   0.0003029356,
   0.0015455309,
   0.0021023897,
   0.0019096405,
   0.0010459052,
  -0.0002873542,
  -0.0018079362,
  -0.0032085717,
  -0.0042155399,
  -0.0046388615,
  -0.0044051939,
  -0.0035675637,
  -0.0022911943,
  -0.0008186191,
   0.0005783862,
   0.0016509290,
   0.0022189865,
   0.0022031496,
   0.0016362665,
   0.0006537423,
  -0.0005353649,
  -0.0016900268,
  -0.0025837822,
  -0.0030468191,
  -0.0029962213,
  -0.0024487116,
  -0.0015144815,
  -0.0003734383,
   0.0007607284,
   0.0016819966,
   0.0022305529,
   0.0023216814,
   0.0019593372,
   0.0012327999,
   0.0002970889,
  -0.0006581181,
  -0.0014453872,
  -0.0019146260,
  -0.0019804399,
  -0.0016362611,
  -0.0009535287,
  -0.0000658210,
   0.0008580397,
   0.0016471432,
   0.0021607311,
   0.0023141058,
   0.0020927990,
   0.0015534665,
   0.0008110094,
   0.0000155174,
  -0.0006772089,
  -0.0011350372,
  -0.0012741051,
  -0.0010731664,
  -0.0005761174,
   0.0001183996,
   0.0008769082,
   0.0015574380,
   0.0020360640,
   0.0022298559,
   0.0021110153,
   0.0017110673,
   0.0011133004,
   0.0004363553,
  -0.0001891703,
  -0.0006451188,
  -0.0008479550,
  -0.0007630664,
  -0.0004103776,
   0.0001410515,
   0.0007863058,
   0.0014049958,
   0.0018830846,
   0.0021343108,
   0.0021151915,
   0.0018330512,
   0.0013432132,
   0.0007385228,
   0.0001308639,
  -0.0003687738,
  -0.0006708376,
  -0.0007217384,
  -0.0005135269,
  -0.0000838414,
   0.0004899964,
   0.0011058786,
   0.0016539740,
   0.0020374502,
   0.0021878813,
   0.0020778532,
   0.0017230476,
   0.0011805651,
   0.0005376049,
  -0.0000998708,
  -0.0006256154,
  -0.0009474820,
  -0.0010061645,
  -0.0007833769,
  -0.0003099051,
   0.0003432160,
   0.0010720145,
   0.0017621006,
   0.0022989373,
   0.0025909626,
   0.0025718063,
   0.0022235882,
   0.0015693929,
   0.0006995893,
  -0.0002656717,
  -0.0011465772,
  -0.0018432517,
   0.0141508892,
};
static float32_t coeffs_delay_361[] = {
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   1.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
   0.0000000000,
};
