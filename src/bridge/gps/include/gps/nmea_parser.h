#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

struct GPSData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double hdop = 0.0;      // Horizontal Dilution of Precision (da GGA)
    double pdop = 0.0;      // Position Dilution of Precision (da GSA)
    double vdop = 0.0;      // Vertical Dilution of Precision (da GSA)
    double speed = 0.0;
    double course = 0.0;
    int satellites = 0;
    int fix_quality = 0;
    std::string timestamp;
    std::string status; // 'A' = valid, 'V' = warning
};

class NMEAParser {
public:
    NMEAParser() = default;
    ~NMEAParser() = default;

    // Processa una riga NMEA e aggiorna i dati GPS
    bool parse(const std::string& nmea_sentence, GPSData& data);

private:
    // Parser per specifiche frasi NMEA
    bool parseGGA(const std::vector<std::string>& tokens, GPSData& data);
    bool parseRMC(const std::vector<std::string>& tokens, GPSData& data);
    bool parseGSA(const std::vector<std::string>& tokens, GPSData& data);
    
    // Converte coordinate NMEA (DDMM.MMMM) in gradi decimali
    double nmeaToDecimal(double nmea_coord, char direction);
    
    // Mappa dei parser per ogni tipo di frase NMEA
    using ParserFunc = bool (NMEAParser::*)(const std::vector<std::string>&, GPSData&);
    std::unordered_map<std::string, ParserFunc> parsers_{};
};

#endif // NMEA_PARSER_H
