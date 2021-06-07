struct {
	uint32_t pCnt = 1;
	uint32_t numI32 = 3;
	uint32_t faults = 0;
} myKeep;

void userHFDebugDump( char *memHF, bool bState ) {
	if ( bState ) {
		myKeep.faults++;
		memcpy(memHF, (void *)&myKeep, sizeof(myKeep));
	}
	else {
		Serial.print("\nFAULT RECOVERY :: userHFDebugDump() in hardfaults.cpp ___ \n");
		memcpy((void *)&myKeep, memHF, sizeof(myKeep));
	}
	return;
}


void setup() {
	Serial.begin(115200);
	while (!Serial && millis() < 4000 );
	Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
	if ( 1 == myKeep.pCnt ) {
		Serial.printf( "%lu,\t%lu\n", 2, myKeep.pCnt );
	}
	else {
		Serial.printf( "RESTART #%lu - last # tested :: %lu,\t%lu\n", myKeep.faults, myKeep.numI32, myKeep.pCnt );
	}
}

void loop() {
	uint32_t diviI32 = 3;
	uint32_t lim = sqrt(myKeep.numI32);
	int flg = 0;
	while ( diviI32 <= lim ) {
		if ( ( myKeep.numI32 % diviI32 )) {
			diviI32 += 2.0;
			continue;
		}
		flg = 1;
		break;
	}
	if ( ! (flg != 0.0) ) {
		myKeep.pCnt++;
		if ( !(myKeep.pCnt % 10000)) {
			Serial.printf( "%lu,\t%lu\n", myKeep.numI32, myKeep.pCnt );
		}
	}
	myKeep.numI32 += 2.0;
	if ( Serial.available() ) {
		int *p = 0;
		*p = 1;
	}
}
