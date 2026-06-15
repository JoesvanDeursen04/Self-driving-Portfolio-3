Technisch Verslag – Autonoom Navigatiesysteem Duckiebot
=======================================================

Vak: AI & Data Science – Opdracht 3
Datum: juni 2026

Inleiding
----------

In deze opdracht is een autonoom navigatiesysteem ontwikkeld voor een
Duckiebot die zelfstandig door een Duckietown-doolhof rijdt. Het systeem
moet een route plannen, zijn positie op de kaart bijhouden, obstakels
herkennen en de robot stabiel door het wegennetwerk sturen. De oplossing is
uitgewerkt binnen ROS en opgesplitst in vier logische onderdelen:
padplanning, lokalisatie, navigatie en regeling.

1. Systeemarchitectuur
----------------------

De architectuur is modulair opgezet. In plaats van alle logica in één node te
plaatsen, is het systeem verdeeld over vier ROS-packages met elk een eigen
verantwoordelijkheid. ``maze_path_planner`` berekent de route, ``maze_localizer``
bepaalt op welke node de robot zich bevindt, ``maze_navigator`` kiest het
benodigde gedrag en ``maze_controller`` zet dat gedrag om naar wielcommando's.

De gegevensstroom is lineair: eerst wordt een pad gepland, daarna wordt de
positie van de robot langs dat pad bijgehouden, vervolgens wordt besloten of
de robot moet rijden, stoppen of afslaan, en ten slotte wordt dat besluit
uitgevoerd door de regelaar.

Concreet betekent dit dat ``maze_path_planner`` het pad en de manoeuvres
publiceert, ``maze_localizer`` daaruit de huidige en volgende node afleidt,
``maze_navigator`` op basis daarvan een rijcommando kiest en
``maze_controller`` dat commando omzet naar stuur- en snelheidsacties voor de
wielen.

Deze scheiding is bewust gekozen. Door de afhankelijkheden tussen de modules
klein te houden blijft het systeem overzichtelijk en is elk onderdeel apart te
testen of te vervangen.

2. Implementatie per taak
--------------------------

Taak 1 – Graafgebaseerde padplanning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voor de padplanning is het doolhof gemodelleerd als een gewogen, ongerichte
graaf. Relevante posities zijn nodes en verbindingen tussen die posities zijn
edges met een kost in tegels. In ``map_definition.py`` is deze kaart als
adjacency list opgeslagen.

De kortste route wordt bepaald met Dijkstra's algoritme in ``dijkstra.py``.
Dat algoritme is gekozen omdat de kaart klein is, alle kosten positief zijn en
de methode eenvoudig en betrouwbaar is. Na het bepalen van het pad berekent de
planner per node ook de benodigde manoeuvre via ``get_maneuver()``. Zowel het
pad als de manoeuvres worden als latched topics gepubliceerd. Voor de
voorbeeldkaart is de kortste route van start naar doel: S -> I -> J -> T.

Taak 2 – Gekalibreerde lokalisatie
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voor lokalisatie is gekozen voor een combinatie van AprilTags en
wielencoder-odometrie. AprilTags geven een absolute positiebepaling wanneer de
camera een tag ziet. Odometrie vult de tussenliggende stukken op door de
afgelegde afstand te schatten met de encoders.

In ``apriltag_localizer.py`` wordt elk tag-ID gekoppeld aan een node op de
kaart. In ``odometry_localizer.py`` worden tickwaarden van beide wielen
omgezet naar een gereden afstand en een geschatte richtingsverandering. In
``maze_localizer_node.py`` worden beide bronnen samengebracht: een geldige
AprilTag-detectie heeft voorrang, en als die ontbreekt gebruikt het systeem de
odometrische schatting. De localizer publiceert de huidige node, de volgende
node en het resterende pad.

Taak 3 – Navigatie
~~~~~~~~~~~~~~~~~~

De navigatielaag vertaalt kaartinformatie naar gedrag. In
``maze_navigator_node.py`` wordt de manoeuvre-lijst vergeleken met de actuele
node, waarna een commando wordt gekozen zoals ``go``, ``left``, ``right`` of
``stop``. De navigator bepaalt dus het gewenste gedrag, maar laat de feitelijke
sturing over aan de controller.

Voor obstakeldetectie gebruikt ``obstacle_detector.py`` een ONNX-model
(``best.onnx``) dat via OpenCV DNN wordt geladen. Als in het onderste deel van
het camerabeeld een duckie met voldoende vertrouwen wordt gedetecteerd, stopt
de robot. Wanneer het model niet beschikbaar is, wordt teruggevallen op
eenvoudige HSV-kleurdetectie. Zodra de doelnode bereikt is, publiceert de
navigator bovendien een ``goal_reached``-signaal en wordt de robot stilgezet.

Taak 4 – Controle met feedback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voor de regelaar is gekozen voor een PID-aanpak. Een PID-regelaar is hier een
goede keuze omdat hij eenvoudig te tunen is, goed uitlegbaar blijft en in de
praktijk voldoende is voor lijnvolging in Duckietown.

De implementatie in ``maze_controller_node.py`` gebruikt de informatie uit het
``lane_pose``-topic als foutsignaal. Daarbij worden laterale afwijking en
hoeksfout gecombineerd. Daarbij wordt de fout berekend als de laterale
afwijking plus 0,15 keer de heading error. De output van de PID-regelaar wordt
omgezet naar differentiële wielsnelheden, zodat de robot kan sturen terwijl
hij vooruit blijft rijden. Voor kruispunten voert de controller tijdelijk een
vast draai-profiel uit bij ``left`` of ``right`` en daarna wordt de gewone
lijnvolging hervat. Ook anti-windup is toegevoegd om te voorkomen dat de
integraalterm te groot oploopt bij verzadiging.

3. Ontwerpkeuzes en afwegingen
-------------------------------

Bij het ontwerp is steeds gekozen voor oplossingen die goed uitlegbaar en
betrouwbaar genoeg zijn voor een demonstratie. Daarom is voor padplanning
Dijkstra gebruikt in plaats van A*: op een kleine, vaste kaart levert dat
vrijwel hetzelfde resultaat op met minder complexiteit. Voor lokalisatie is
bewust niet alleen op odometrie vertrouwd, omdat encoderfouten zich kunnen
opstapelen; AprilTags geven hier een absolute correctie. Voor obstakels is
gekozen voor een ONNX-model met HSV-fallback, zodat nauwkeurigheid wordt
gecombineerd met een eenvoudige reserveoplossing. Bij de regeling is een PID
gebruikt in plaats van een complexere methode zoals reinforcement learning,
omdat PID direct inzetbaar en eenvoudiger af te stellen is.

4. Beperkingen en faalscenario's
---------------------------------

De belangrijkste beperking is dat het systeem sterk afhankelijk is van de
vooraf ingestelde kaart. Zowel de graaf als de AprilTag-mapping moeten
overeenkomen met de echte demonstratieopstelling. Als de kaart tijdens de
demonstratie anders blijkt te zijn dan verwacht, kan de planner nog steeds een
geldig pad berekenen, maar dan wel over een verkeerde structuur. In de
praktijk betekent dit dat de configuratie vóór de demo goed gecontroleerd moet
worden.

Ook de lokalisatie heeft duidelijke grenzen. Zodra een AprilTag even niet
zichtbaar is, moet het systeem terugvallen op odometrie. Dat werkt goed op
korte stukken, maar encoderfouten stapelen zich langzaam op door wielslip,
ongelijke ondergrond of kleine meetonnauwkeurigheden. Vooral na een bocht of
na een langere rechte strook kan de geschatte positie dus afwijken van de
werkelijkheid. Als de fout te groot wordt, kan de robot denken dat hij al op
de volgende node staat terwijl dat fysiek nog niet zo is.

De obstakeldetectie is eveneens functioneel maar beperkt. De robot kan een
duckie herkennen en veilig stoppen, maar hij kan niet zelfstandig een
alternatieve route om het obstakel heen plannen. Daarmee is de oplossing goed
geschikt voor het vermijden van botsingen, maar niet voor dynamische
herplanning in een druk of veranderlijk doolhof. Als een duckie precies op een
kritieke plek blijft staan, zal de robot wachten tot het pad vrij is.

Bij de regeling ligt het grootste risico in de afhankelijkheid van de
lijnwaarneming. Als ``lane_filter_node`` een onbetrouwbaar foutsignaal levert
door slechte belichting, reflecties of schaduw, kan de PID-regelaar te sterk of
juist te zwak corrigeren. Daardoor kan de robot gaan slingeren, de middenlijn
verliezen of in extreme gevallen zelfs uit de baan raken. De vaste bochttimer
maakt het systeem bovendien gevoelig voor variaties in batterijspanning,
wielslip en wrijving. Een iets lagere snelheid kan er dan al voor zorgen dat
een bocht niet mooi wordt uitgekomen.

Een laatste beperking is dat het huidige systeem vooral gericht is op een
vooraf bekende demonstratie. Er is geen algemene foutafhandeling voor ontbreke
nde topics, onverwachte sensoruitval of een volledig andere kaartlayout.
Daarom is dit systeem vooral geschikt als gecontroleerde demonstratieoplossing
en minder als volledig robuuste productierijder. Tegelijk laat het wel zien
hoe padplanning, lokalisatie, navigatie en regeling samen een werkend
autonoom systeem vormen.
