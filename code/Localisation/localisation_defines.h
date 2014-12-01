#pragma once
#include <array>


#define PI	3.141592

#define LINE1_DISTANCE			99
#define LINE1_ANGLE				0.1
#define LINE2_DISTANCE			120
#define LINE2_ANGLE				0.2
#define LINE3_DISTANCE			125
#define LINE3_ANGLE				0.3
#define LINE4_DISTANCE			85
#define LINE4_ANGLE				0.4

#define ANGLE_DEVIATION_MAX		2.0
#define ANGLE_INDEX_NOMATCH		287


#define ROBOT_START_DISTANCE	119
#define ROBOT_START_ANGLE		0.1
#define ROBOT_START_ORIENTATION	3.10

using namespace std;



const array<double,681> cosarr = {{
													 -0.707106781186548,-0.702189602654915,-0.697238701830508,-0.692254316477780,-0.687236685969263,-0.682186051274067,-0.677102654946315,-0.671986741113487,-0.666838555464701,-0.661658345238914,-0.656446359213043,-0.651202847690023,-0.645928062486788,-0.640622256922168,-0.635285685804734,-0.629918605420557,-0.624521273520899,-0.619093949309834,-0.613636893431803,-0.608150367959093,-0.602634636379256,-0.597089963582450,-0.591516615848721,-0.585914860835213,-0.580284967563316,-0.574627206405744,-0.568941849073553,-0.563229168603091,-0.557489439342885,-0.551722936940467,-0.545929938329134,-0.540110721714651,-0.534265566561888,-0.528394753581401,-0.522498564715949,-0.516577283126955,-0.510631193180907,-0.504660580435702,-0.498665731626932,-0.492646934654112,-0.486604478566856,-0.480538653550996,-0.474449750914643,-0.468338063074198,-0.462203883540313,-0.456047506903789,-0.449869228821433,-0.443669346001856,-0.437448156191226,-0.431205958158970,-0.424943051683422,-0.418659737537428,-0.412356317473903,-0.406033094211339,-0.399690371419264,-0.393328453703663,-0.386947646592345,-0.380548256520276,-0.374130590814855,-0.367694957681163,-0.361241666187153,-0.354771026248815,-0.348283348615289,-0.341778944853938,-0.335258127335394,-0.328721209218546,-0.322168504435510,-0.315600327676546,-0.309016994374947,-0.302418820691895,-0.295806123501270,-0.289179220374438,-0.282538429564995,-0.275884069993489,-0.269216461232099,-0.262535923489291,-0.255842777594436,-0.249137344982407,-0.242419947678140,-0.235690908281169,-0.228950549950134,-0.222199196387260,-0.215437171822812,-0.208664800999526,-0.201882409157010,-0.195090322016128,-0.188288865763355,-0.181478367035111,-0.174659152902079,-0.167831550853493,-0.160995888781413,-0.154152494964977,-0.147301698054637,-0.140443827056374,-0.133579211315899,-0.126708180502834,-0.119831064594883,-0.112948193861985,-0.106059898850448,-0.0991665103670801,-0.0922683594633020,-0.0853657774192453,-0.0784590957278449,-0.0715486460789202,-0.0646347603432442,-0.0577177705566062,-0.0507980089038656,-0.0438758077030002,-0.0369514993891450,-0.0300254164986280,-0.0230978916530009,-0.0161692575430647,-0.00923984691289084,-0.00230999254384413,0.00461997276140043,0.0115497161948407,0.0184789049591300,0.0254072062835582,0.0323342874400352,0.0392598157590686,0.0461834586457397,0.0531048835956769,0.0600237582110241,0.0669397502164039,0.0738525274748740,0.0807617580038792,0.0876671099911944,0.0945682518108589,0.101464852039103,0.108356579470265,0.115243103132698,0.122124092304660,0.128999216530203,0.135868145635040,0.142730549742399,0.149586099288871,0.156434465040231,0.163275318107253,0.170108329961504,0.176933172451121,0.183749517816570,0.190557038706389,0.197355408192904,0.204144299787935,0.210923387458470,0.217692345642328,0.224450849263791,0.231198573749215,0.237935195042619,0.244660389621246,0.251373834511103,0.258075207302468,0.264764186165376,0.271440449865074,0.278103677777449,0.284753549904423,0.291389746889325,0.298011950032223,0.304619841305234,0.311213103367794,0.317791419581902,0.324354474027319,0.330901951516748,0.337433537610967,0.343948918633928,0.350447781687826,0.356929814668120,0.363394706278528,0.369842146045969,0.376271824335482,0.382683432365090,0.389076662220630,0.395451206870543,0.401806760180615,0.408143016928686,0.414459672819297,0.420756424498315,0.427032969567495,0.433289006599002,0.439524235149893,0.445738355776538,0.451931070049006,0.458102080565393,0.464251090966110,0.470377805948109,0.476481931279070,0.482563173811529,0.488621241496955,0.494655843399779,0.500666689711363,0.506653491763919,0.512615962044374,0.518553814208172,0.524466763093033,0.530354524732642,0.536216816370290,0.542053356472449,0.547863864742299,0.553648062133182,0.559405670862007,0.565136414422592,0.570840017598938,0.576516206478450,0.582164708465092,0.587785252292473,0.593377568036882,0.598941387130246,0.604476442373026,0.609982467947057,0.615459199428305,0.620906373799571,0.626323729463122,0.631711006253251,0.637067945448774,0.642394289785456,0.647689783468362,0.652954172184144,0.658187203113256,0.663388624942091,0.668558187875054,0.673695643646557,0.678800745532942,0.683873248364328,0.688912908536389,0.693919484022050,0.698892734383110,0.703832420781790,0.708738305992205,0.713610154411752,0.718447732072430,0.723250806652070,0.728019147485499,0.732752525575611,0.737450713604372,0.742113485943728,0.746740618666448,0.751331889556873,0.755887078121593,0.760405965600031,0.764888334974950,0.769333970982879,0.773742660124445,0.778114190674630,0.782448352692940,0.786744938033483,0.791003740354969,0.795224555130617,0.799407179657979,0.803551413068674,0.807657056338033,0.811723912294661,0.815751785629902,0.819740482907221,0.823689812571492,0.827599584958200,0.831469612302545,0.835299708748465,0.839089690357558,0.842839375117915,0.846548582952864,0.850217135729614,0.853844857267816,0.857431573348016,0.860977111720030,0.864481302111208,0.867943976234618,0.871364967797126,0.874744112507378,0.878081248083698,0.881376214261873,0.884628852802855,0.887839007500361,0.891006524188368,0.894131250748526,0.897213037117455,0.900251735293960,0.903247199346129,0.906199285418350,0.909107851738215,0.911972758623330,0.914793868488021,0.917571045849946,0.920304157336598,0.922993071691708,0.925637659781556,0.928237794601165,0.930793351280404,0.933304207089984,0.935770241447353,0.938191335922484,0.940567374243568,0.942898242302591,0.945183828160820,0.947424022054174,0.949618716398501,0.951767805794738,0.953871187033978,0.955928759102424,0.957940423186240,0.959906082676299,0.961825643172819,0.963699012489900,0.965526100659948,0.967306819937998,0.969041084805928,0.970728811976562,0.972369920397677,0.973964331255887,0.975511967980437,0.977012756246871,0.978466623980609,0.979873501360405,0.981233320821699,0.982546017059864,0.983811527033342,0.985029789966671,0.986200747353403,0.987324342958914,0.988400522823106,0.989429235262998,0.990410430875205,0.991344062538317,0.992230085415154,0.993068456954926,0.993859136895274,0.994602087264201,0.995297272381900,0.995944658862465,0.996544215615495,0.997095913847586,0.997599727063717,0.998055631068518,0.998463603967434,0.998823626167778,0.999135680379671,0.999399751616869,0.999615827197487,0.999783896744609,0.999903952186781,0.999975987758401,1,0.999975987758401,0.999903952186781,0.999783896744609,0.999615827197487,0.999399751616869,0.999135680379671,0.998823626167778,0.998463603967434,0.998055631068518,0.997599727063717,0.997095913847586,0.996544215615495,0.995944658862465,0.995297272381900,0.994602087264201,0.993859136895274,0.993068456954926,0.992230085415154,0.991344062538317,0.990410430875205,0.989429235262998,0.988400522823106,0.987324342958914,0.986200747353403,0.985029789966671,0.983811527033342,0.982546017059864,0.981233320821699,0.979873501360405,0.978466623980609,0.977012756246871,0.975511967980437,0.973964331255887,0.972369920397677,0.970728811976562,0.969041084805928,0.967306819937998,0.965526100659948,0.963699012489900,0.961825643172819,0.959906082676299,0.957940423186240,0.955928759102424,0.953871187033978,0.951767805794738,0.949618716398501,0.947424022054174,0.945183828160820,0.942898242302591,0.940567374243568,0.938191335922484,0.935770241447353,0.933304207089984,0.930793351280404,0.928237794601165,0.925637659781556,0.922993071691708,0.920304157336598,0.917571045849946,0.914793868488021,0.911972758623330,0.909107851738215,0.906199285418350,0.903247199346129,0.900251735293960,0.897213037117455,0.894131250748526,0.891006524188368,0.887839007500360,0.884628852802855,0.881376214261873,0.878081248083698,0.874744112507378,0.871364967797126,0.867943976234618,0.864481302111208,0.860977111720030,0.857431573348016,0.853844857267816,0.850217135729614,0.846548582952864,0.842839375117915,0.839089690357558,0.835299708748466,0.831469612302545,0.827599584958200,0.823689812571492,0.819740482907221,0.815751785629902,0.811723912294661,0.807657056338033,0.803551413068674,0.799407179657979,0.795224555130617,0.791003740354969,0.786744938033483,0.782448352692940,0.778114190674630,0.773742660124445,0.769333970982879,0.764888334974951,0.760405965600031,0.755887078121593,0.751331889556874,0.746740618666448,0.742113485943728,0.737450713604372,0.732752525575611,0.728019147485499,0.723250806652070,0.718447732072430,0.713610154411752,0.708738305992205,0.703832420781790,0.698892734383110,0.693919484022050,0.688912908536389,0.683873248364328,0.678800745532942,0.673695643646557,0.668558187875054,0.663388624942091,0.658187203113256,0.652954172184144,0.647689783468362,0.642394289785456,0.637067945448774,0.631711006253251,0.626323729463122,0.620906373799571,0.615459199428305,0.609982467947057,0.604476442373026,0.598941387130245,0.593377568036882,0.587785252292473,0.582164708465092,0.576516206478450,0.570840017598938,0.565136414422592,0.559405670862007,0.553648062133182,0.547863864742299,0.542053356472449,0.536216816370290,0.530354524732642,0.524466763093033,0.518553814208172,0.512615962044374,0.506653491763919,0.500666689711363,0.494655843399779,0.488621241496955,0.482563173811529,0.476481931279071,0.470377805948109,0.464251090966110,0.458102080565393,0.451931070049006,0.445738355776538,0.439524235149893,0.433289006599003,0.427032969567495,0.420756424498315,0.414459672819297,0.408143016928686,0.401806760180615,0.395451206870542,0.389076662220630,0.382683432365090,0.376271824335482,0.369842146045969,0.363394706278528,0.356929814668120,0.350447781687826,0.343948918633928,0.337433537610967,0.330901951516748,0.324354474027319,0.317791419581902,0.311213103367794,0.304619841305234,0.298011950032223,0.291389746889325,0.284753549904423,0.278103677777449,0.271440449865074,0.264764186165376,0.258075207302468,0.251373834511103,0.244660389621246,0.237935195042619,0.231198573749215,0.224450849263791,0.217692345642328,0.210923387458470,0.204144299787935,0.197355408192904,0.190557038706389,0.183749517816570,0.176933172451121,0.170108329961504,0.163275318107253,0.156434465040231,0.149586099288871,0.142730549742400,0.135868145635040,0.128999216530203,0.122124092304660,0.115243103132698,0.108356579470265,0.101464852039103,0.0945682518108589,0.0876671099911944,0.0807617580038792,0.0738525274748742,0.0669397502164039,0.0600237582110241,0.0531048835956766,0.0461834586457397,0.0392598157590686,0.0323342874400352,0.0254072062835585,0.0184789049591300,0.0115497161948407,0.00461997276140018,-0.00230999254384413,-0.00923984691289084,-0.0161692575430647,-0.0230978916530009,-0.0300254164986280,-0.0369514993891450,-0.0438758077030000,-0.0507980089038656,-0.0577177705566062,-0.0646347603432445,-0.0715486460789202,-0.0784590957278449,-0.0853657774192453,-0.0922683594633018,-0.0991665103670801,-0.106059898850448,-0.112948193861985,-0.119831064594883,-0.126708180502834,-0.133579211315899,-0.140443827056374,-0.147301698054637,-0.154152494964977,-0.160995888781413,-0.167831550853493,-0.174659152902079,-0.181478367035111,-0.188288865763355,-0.195090322016128,-0.201882409157010,-0.208664800999526,-0.215437171822812,-0.222199196387260,-0.228950549950134,-0.235690908281169,-0.242419947678140,-0.249137344982407,-0.255842777594435,-0.262535923489291,-0.269216461232099,-0.275884069993489,-0.282538429564995,-0.289179220374438,-0.295806123501271,-0.302418820691895,-0.309016994374947,-0.315600327676546,-0.322168504435510,-0.328721209218546,-0.335258127335394,-0.341778944853939,-0.348283348615289,-0.354771026248815,-0.361241666187153,-0.367694957681163,-0.374130590814855,-0.380548256520276,-0.386947646592345,-0.393328453703663,-0.399690371419264,-0.406033094211339,-0.412356317473903,-0.418659737537428,-0.424943051683422,-0.431205958158970,-0.437448156191226,-0.443669346001856,-0.449869228821433,-0.456047506903789,-0.462203883540313,-0.468338063074198,-0.474449750914642,-0.480538653550996,-0.486604478566856,-0.492646934654112,-0.498665731626932,-0.504660580435702,-0.510631193180907,-0.516577283126955,-0.522498564715949,-0.528394753581401,-0.534265566561889,-0.540110721714651,-0.545929938329134,-0.551722936940468,-0.557489439342885,-0.563229168603091,-0.568941849073553,-0.574627206405744,-0.580284967563316,-0.585914860835213,-0.591516615848721,-0.597089963582451,-0.602634636379256,-0.608150367959093,-0.613636893431803,-0.619093949309834,-0.624521273520899,-0.629918605420558,-0.635285685804734,-0.640622256922167,-0.645928062486788,-0.651202847690023,-0.656446359213042,-0.661658345238914,-0.666838555464701,-0.671986741113487,-0.677102654946315,-0.682186051274067,-0.687236685969263,-0.692254316477780,-0.697238701830508,-0.702189602654915,-0.707106781186548
												 }};

const array<double,681> sinarr = {{
													 -0.707106781186548,-0.711990001280449,-0.716839028422496,-0.721653629740668,-0.726433574016224,-0.731178631694811,-0.735888574897485,-0.740563177431660,-0.745202214801963,-0.749805464221022,-0.754372704620164,-0.758903716660028,-0.763398282741103,-0.767856187014176,-0.772277215390697,-0.776661155553064,-0.781007796964815,-0.785316930880745,-0.789588350356923,-0.793821850260636,-0.798017227280240,-0.802174279934920,-0.806292808584373,-0.810372615438388,-0.814413504566349,-0.818415281906644,-0.822377755275984,-0.826300734378635,-0.830184030815551,-0.834027458093428,-0.837830831633658,-0.841593968781192,-0.845316688813314,-0.848998812948317,-0.852640164354092,-0.856240568156621,-0.859799851448373,-0.863317843296604,-0.866794374751577,-0.870229278854663,-0.873622390646370,-0.876973547174255,-0.880282587500760,-0.883549352710933,-0.886773685920062,-0.889955432281212,-0.893094438992657,-0.896190555305224,-0.899243632529526,-0.902253524043106,-0.905220085297482,-0.908143173825081,-0.911022649246088,-0.913858373275184,-0.916650209728188,-0.919398024528597,-0.922101685714024,-0.924761063442540,-0.927376029998904,-0.929946459800696,-0.932472229404356,-0.934953217511102,-0.937389304972764,-0.939780374797499,-0.942126312155416,-0.944427004384085,-0.946682340993951,-0.948892213673638,-0.951056516295154,-0.953175144918983,-0.955247997799080,-0.957274975387758,-0.959255980340465,-0.961190917520462,-0.963079694003391,-0.964922219081738,-0.966718404269187,-0.968468163304875,-0.970171412157526,-0.971828069029495,-0.973438054360693,-0.975001290832405,-0.976517703371008,-0.977987219151574,-0.979409767601366,-0.980785280403230,-0.982113691498876,-0.983394937092046,-0.984628955651584,-0.985815687914385,-0.986955076888246,-0.988047067854600,-0.989091608371146,-0.990088648274365,-0.991038139681931,-0.991940036995010,-0.992794296900449,-0.993600878372858,-0.994359742676579,-0.995070853367546,-0.995734176295035,-0.996349679603305,-0.996917333733128,-0.997437111423208,-0.997908987711490,-0.998332939936360,-0.998708947737729,-0.999036993058020,-0.999317060143023,-0.999549135542663,-0.999733208111637,-0.999869269009957,-0.999957311703368,-0.999997331963665,-0.999989327868895,-0.999933299803451,-0.999829250458053,-0.999677184829615,-0.999477110221012,-0.999229036240723,-0.998932974802372,-0.998588940124158,-0.998196948728168,-0.997757019439586,-0.997269173385788,-0.996733433995330,-0.996149826996819,-0.995518380417679,-0.994839124582806,-0.994112092113110,-0.993337317923950,-0.992514839223456,-0.991644695510743,-0.990726928574012,-0.989761582488547,-0.988748703614594,-0.987688340595138,-0.986580544353564,-0.985425368091216,-0.984222867284835,-0.982973099683902,-0.981676125307859,-0.980332006443232,-0.978940807640633,-0.977502595711666,-0.976017439725716,-0.974485411006631,-0.972906583129300,-0.971281031916114,-0.969608835433331,-0.967890073987323,-0.966124830120719,-0.964313188608445,-0.962455236453647,-0.960551062883518,-0.958600759345010,-0.956604419500441,-0.954562139223001,-0.952474016592145,-0.950340151888884,-0.948160647590966,-0.945935608367959,-0.943665141076223,-0.941349354753775,-0.938988360615057,-0.936582272045591,-0.934131204596539,-0.931635275979148,-0.929094606059099,-0.926509316850752,-0.923879532511287,-0.921205379334736,-0.918486985745923,-0.915724482294297,-0.912918001647657,-0.910067678585786,-0.907173649993977,-0.904236054856456,-0.901255034249712,-0.898230731335719,-0.895163291355062,-0.892052861619961,-0.888899591507195,-0.885703632450933,-0.882465137935456,-0.879184263487790,-0.875861166670234,-0.872496007072797,-0.869088946305528,-0.865640147990761,-0.862149777755251,-0.858618003222225,-0.855044994003331,-0.851430921690490,-0.847775959847655,-0.844080284002482,-0.840344071637893,-0.836567502183555,-0.832750757007265,-0.828894019406237,-0.824997474598302,-0.821061309713011,-0.817085713782647,-0.813070877733150,-0.809016994374948,-0.804924258393692,-0.800792866340914,-0.796623016624583,-0.792414909499574,-0.788168747058059,-0.783884733219794,-0.779563073722329,-0.775203976111130,-0.770807649729605,-0.766374305709057,-0.761904156958542,-0.757397418154643,-0.752854305731162,-0.748275037868725,-0.743659834484305,-0.739008917220659,-0.734322509435686,-0.729600836191696,-0.724844124244608,-0.720052602033055,-0.715226499667413,-0.710366048918757,-0.705471483207720,-0.700543037593291,-0.695580948761524,-0.690585455014171,-0.685556796257238,-0.680495213989461,-0.675400951290715,-0.670274252810332,-0.665115364755357,-0.659924534878723,-0.654702012467352,-0.649448048330184,-0.644162894786131,-0.638846805651961,-0.633500036230108,-0.628122843296409,-0.622715485087776,-0.617278221289793,-0.611811313024243,-0.606315022836572,-0.600789614683273,-0.595235353919219,-0.589652507284912,-0.584041342893677,-0.578402130218783,-0.572735140080505,-0.567040644633116,-0.561318917351817,-0.555570233019602,-0.549794867714067,-0.543993098794146,-0.538165204886792,-0.532311465873598,-0.526432162877356,-0.520527578248553,-0.514597995551815,-0.508643699552286,-0.502664976201954,-0.496662112625918,-0.490635397108600,-0.484585119079898,-0.478511569101287,-0.472415038851865,-0.466295821114348,-0.460154209761005,-0.453990499739547,-0.447804987058962,-0.441597968775301,-0.435369742977408,-0.429120608772609,-0.422850866272344,-0.416560816577757,-0.410250761765234,-0.403921004871895,-0.397571849881045,-0.391203601707570,-0.384816566183298,-0.378411050042310,-0.371987360906208,-0.365545807269341,-0.359086698483996,-0.352610344745533,-0.346117057077493,-0.339607147316662,-0.333080928098090,-0.326538712840083,-0.319980815729149,-0.313407551704906,-0.306819236444962,-0.300216186349753,-0.293598718527347,-0.286967150778216,-0.280321801579974,-0.273662990072083,-0.266991036040523,-0.260306259902439,-0.253608982690750,-0.246899526038733,-0.240178212164576,-0.233445363855905,-0.226701304454280,-0.219946357839669,-0.213180848414891,-0.206405101090040,-0.199619441266880,-0.192824194823217,-0.186019688097249,-0.179206247871897,-0.172384201359104,-0.165553876184130,-0.158715600369811,-0.151869702320808,-0.145016510807839,-0.138156354951882,-0.131289564208379,-0.124416468351405,-0.117537397457838,-0.110652681891501,-0.103762652287302,-0.0968676395353515,-0.0899679747650738,-0.0830639893293037,-0.0761560147883742,-0.0692443828941916,-0.0623294255743062,-0.0554114749159701,-0.0484908631501875,-0.0415679226357623,-0.0346429858433354,-0.0277163853394164,-0.0207884537704155,-0.0138595238466657,-0.00692992832644633,0,0.00692992832644633,0.0138595238466657,0.0207884537704155,0.0277163853394164,0.0346429858433354,0.0415679226357623,0.0484908631501875,0.0554114749159701,0.0623294255743062,0.0692443828941916,0.0761560147883742,0.0830639893293037,0.0899679747650738,0.0968676395353515,0.103762652287302,0.110652681891501,0.117537397457838,0.124416468351405,0.131289564208379,0.138156354951882,0.145016510807839,0.151869702320809,0.158715600369811,0.165553876184130,0.172384201359104,0.179206247871897,0.186019688097249,0.192824194823217,0.199619441266879,0.206405101090040,0.213180848414891,0.219946357839669,0.226701304454280,0.233445363855905,0.240178212164577,0.246899526038733,0.253608982690750,0.260306259902439,0.266991036040523,0.273662990072083,0.280321801579974,0.286967150778216,0.293598718527346,0.300216186349753,0.306819236444962,0.313407551704906,0.319980815729149,0.326538712840083,0.333080928098090,0.339607147316661,0.346117057077493,0.352610344745533,0.359086698483996,0.365545807269341,0.371987360906208,0.378411050042310,0.384816566183298,0.391203601707570,0.397571849881045,0.403921004871895,0.410250761765234,0.416560816577757,0.422850866272344,0.429120608772609,0.435369742977408,0.441597968775301,0.447804987058962,0.453990499739547,0.460154209761005,0.466295821114348,0.472415038851865,0.478511569101287,0.484585119079898,0.490635397108600,0.496662112625918,0.502664976201954,0.508643699552285,0.514597995551815,0.520527578248553,0.526432162877356,0.532311465873598,0.538165204886792,0.543993098794146,0.549794867714067,0.555570233019602,0.561318917351817,0.567040644633116,0.572735140080505,0.578402130218783,0.584041342893677,0.589652507284912,0.595235353919219,0.600789614683273,0.606315022836571,0.611811313024243,0.617278221289793,0.622715485087776,0.628122843296409,0.633500036230108,0.638846805651962,0.644162894786131,0.649448048330184,0.654702012467352,0.659924534878722,0.665115364755357,0.670274252810332,0.675400951290715,0.680495213989461,0.685556796257238,0.690585455014172,0.695580948761524,0.700543037593291,0.705471483207720,0.710366048918757,0.715226499667413,0.720052602033055,0.724844124244608,0.729600836191696,0.734322509435686,0.739008917220659,0.743659834484305,0.748275037868725,0.752854305731162,0.757397418154643,0.761904156958542,0.766374305709057,0.770807649729605,0.775203976111130,0.779563073722329,0.783884733219794,0.788168747058059,0.792414909499574,0.796623016624583,0.800792866340915,0.804924258393692,0.809016994374948,0.813070877733150,0.817085713782646,0.821061309713011,0.824997474598302,0.828894019406237,0.832750757007265,0.836567502183555,0.840344071637893,0.844080284002482,0.847775959847655,0.851430921690490,0.855044994003331,0.858618003222225,0.862149777755251,0.865640147990761,0.869088946305528,0.872496007072797,0.875861166670235,0.879184263487790,0.882465137935456,0.885703632450933,0.888899591507195,0.892052861619961,0.895163291355062,0.898230731335720,0.901255034249712,0.904236054856456,0.907173649993977,0.910067678585786,0.912918001647657,0.915724482294297,0.918486985745923,0.921205379334736,0.923879532511287,0.926509316850753,0.929094606059099,0.931635275979148,0.934131204596539,0.936582272045591,0.938988360615057,0.941349354753775,0.943665141076223,0.945935608367959,0.948160647590966,0.950340151888884,0.952474016592145,0.954562139223001,0.956604419500441,0.958600759345010,0.960551062883518,0.962455236453647,0.964313188608445,0.966124830120719,0.967890073987323,0.969608835433331,0.971281031916114,0.972906583129300,0.974485411006631,0.976017439725716,0.977502595711666,0.978940807640633,0.980332006443232,0.981676125307859,0.982973099683902,0.984222867284835,0.985425368091216,0.986580544353564,0.987688340595138,0.988748703614594,0.989761582488547,0.990726928574012,0.991644695510743,0.992514839223456,0.993337317923950,0.994112092113110,0.994839124582806,0.995518380417679,0.996149826996819,0.996733433995330,0.997269173385788,0.997757019439586,0.998196948728168,0.998588940124158,0.998932974802372,0.999229036240723,0.999477110221012,0.999677184829615,0.999829250458053,0.999933299803451,0.999989327868895,0.999997331963665,0.999957311703368,0.999869269009957,0.999733208111637,0.999549135542663,0.999317060143023,0.999036993058020,0.998708947737729,0.998332939936360,0.997908987711490,0.997437111423208,0.996917333733128,0.996349679603305,0.995734176295035,0.995070853367546,0.994359742676579,0.993600878372858,0.992794296900449,0.991940036995010,0.991038139681931,0.990088648274365,0.989091608371146,0.988047067854600,0.986955076888246,0.985815687914385,0.984628955651584,0.983394937092046,0.982113691498876,0.980785280403230,0.979409767601366,0.977987219151574,0.976517703371008,0.975001290832405,0.973438054360693,0.971828069029495,0.970171412157526,0.968468163304875,0.966718404269188,0.964922219081738,0.963079694003391,0.961190917520462,0.959255980340465,0.957274975387758,0.955247997799080,0.953175144918983,0.951056516295154,0.948892213673638,0.946682340993951,0.944427004384085,0.942126312155416,0.939780374797499,0.937389304972764,0.934953217511102,0.932472229404356,0.929946459800696,0.927376029998904,0.924761063442540,0.922101685714025,0.919398024528597,0.916650209728188,0.913858373275184,0.911022649246088,0.908143173825081,0.905220085297482,0.902253524043107,0.899243632529526,0.896190555305224,0.893094438992657,0.889955432281212,0.886773685920062,0.883549352710933,0.880282587500760,0.876973547174255,0.873622390646370,0.870229278854663,0.866794374751577,0.863317843296604,0.859799851448372,0.856240568156621,0.852640164354092,0.848998812948317,0.845316688813314,0.841593968781192,0.837830831633659,0.834027458093428,0.830184030815551,0.826300734378635,0.822377755275984,0.818415281906644,0.814413504566348,0.810372615438388,0.806292808584373,0.802174279934920,0.798017227280240,0.793821850260636,0.789588350356923,0.785316930880745,0.781007796964816,0.776661155553063,0.772277215390697,0.767856187014176,0.763398282741103,0.758903716660028,0.754372704620164,0.749805464221022,0.745202214801963,0.740563177431659,0.735888574897485,0.731178631694811,0.726433574016224,0.721653629740668,0.716839028422496,0.711990001280448,0.707106781186548
												 }};

