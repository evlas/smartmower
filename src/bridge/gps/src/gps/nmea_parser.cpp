#include "gps/nmea_parser.h"
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace std;

bool NMEAParser::parse(const string& nmea_sentence, GPSData& data) {
    if (nmea_sentence.empty() || nmea_sentence[0] != '$') {
        return false;
    }

    // Verifica il checksum se presente
    size_t checksum_pos = nmea_sentence.find('*');
    if (checksum_pos != string::npos) {
        // Calcola il checksum
        uint8_t calculated_checksum = 0;
        for (size_t i = 1; i < checksum_pos; i++) {
            calculated_checksum ^= nmea_sentence[i];
        }
        
        // Estrai il checksum dalla stringa
        uint8_t received_checksum;
        stringstream ss(nmea_sentence.substr(checksum_pos + 1, 2));
        ss >> hex >> received_checksum;
        
        if (calculated_checksum != received_checksum) {
            return false; // Checksum non valido
        }
    }

    // Estrai il tipo di frase NMEA (es. $GPGGA -> GPGGA)
    size_t start = nmea_sentence.find('$') + 1;
    size_t end = nmea_sentence.find(',');
    if (end == string::npos) {
        return false;
    }
    
    string nmea_type = nmea_sentence.substr(start, end - start);
    
    // Tokenizza la stringa
    vector<string> tokens;
    string token;
    istringstream token_stream(nmea_sentence.substr(end + 1));
    
    while (getline(token_stream, token, ',')) {
        tokens.push_back(token);
    }
    
    // Chiama il parser appropriato in base al tipo di frase
    if (nmea_type == "GPGGA") {
        return parseGGA(tokens, data);
    } else if (nmea_type == "GPRMC") {
        return parseRMC(tokens, data);
    }
    
    return false;
}

bool NMEAParser::parseGGA(const vector<string>& tokens, GPSData& data) {
    if (tokens.size() < 10) {
        return false;
    }
    
    try {
        // Ora UTC
        if (!tokens[0].empty()) {
            data.timestamp = tokens[0].substr(0, 2) + ":" + 
                            tokens[0].substr(2, 2) + ":" + 
                            tokens[0].substr(4, 2);
        }
        
        // Latitudine
        if (!tokens[1].empty() && !tokens[2].empty()) {
            double lat = stod(tokens[1]);
            data.latitude = nmeaToDecimal(lat, tokens[2][0]);
        }
        
        // Longitudine
        if (!tokens[3].empty() && !tokens[4].empty()) {
            double lon = stod(tokens[3]);
            data.longitude = nmeaToDecimal(lon, tokens[4][0]);
        }
        
        // Qualità del fix
        if (!tokens[5].empty()) {
            data.fix_quality = stoi(tokens[5]);
        }
        
        // Numero di satelliti
        if (!tokens[6].empty()) {
            data.satellites = stoi(tokens[6]);
        }
        
        // Altitudine
        if (!tokens[8].empty()) {
            data.altitude = stod(tokens[8]);
        }
        
        return true;
    } catch (const exception&) {
        return false;
    }
}

bool NMEAParser::parseRMC(const vector<string>& tokens, GPSData& data) {
    if (tokens.size() < 10) {
        return false;
    }
    
    try {
        // Stato della posizione
        if (!tokens[1].empty()) {
            data.status = tokens[1];
        }
        
        // Latitudine
        if (!tokens[2].empty() && !tokens[3].empty()) {
            double lat = stod(tokens[2]);
            data.latitude = nmeaToDecimal(lat, tokens[3][0]);
        }
        
        // Longitudine
        if (!tokens[4].empty() && !tokens[5].empty()) {
            double lon = stod(tokens[4]);
            data.longitude = nmeaToDecimal(lon, tokens[5][0]);
        }
        
        // Velocità in nodi
        if (!tokens[6].empty()) {
            data.speed = stod(tokens[6]) * 1.852; // Converti in km/h
        }
        
        // Angolo di rotta
        if (!tokens[7].empty()) {
            data.course = stod(tokens[7]);
        }
        
        // Data (se presente)
        if (tokens.size() > 8 && !tokens[8].empty()) {
            // Aggiungi la data al timestamp se non è vuoto
            if (!data.timestamp.empty()) {
                data.timestamp += " " + tokens[8].substr(4, 2) + "/" + 
                                 tokens[8].substr(2, 2) + "/20" + 
                                 tokens[8].substr(0, 2);
            }
        }
        
        return true;
    } catch (const exception&) {
        return false;
    }
}

double NMEAParser::nmeaToDecimal(double nmea_coord, char direction) {
    // Converte da GGMM.MMMM a GG.DDDDDD
    double degrees = floor(nmea_coord / 100.0);
    double minutes = nmea_coord - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    
    // Applica il segno in base alla direzione
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}
