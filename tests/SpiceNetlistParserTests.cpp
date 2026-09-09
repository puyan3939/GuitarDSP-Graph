#include "guitardsp/circuit/SpiceNetlistParser.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

bool nearlyEqual(double a, double b, double relTol = 1.0e-9) {
    return std::abs(a - b) <= relTol * std::max(1.0, std::abs(b));
}

// Parses `text` and expects it to fail; returns the thrown error's message
// (via SpiceParseError::what()), or empty if it unexpectedly succeeded.
std::string expectError(const std::string& text) {
    try {
        (void)circuit::parseSpiceNetlist(text);
    } catch (const circuit::SpiceParseError& e) {
        return e.what();
    }
    return {};
}
}

int main() {
    bool ok = true;

    // --- Numeric magnitude suffixes: the M vs MEG trap called out by issue #99 ---
    {
        bool parseOk = false;
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1M", parseOk), 1.0e-3) && parseOk,
                      "suffix 'M' means milli (1e-3), not mega");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1MEG", parseOk), 1.0e6) && parseOk,
                      "suffix 'MEG' means mega (1e6)");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1meg", parseOk), 1.0e6) && parseOk,
                      "suffix matching is case-insensitive ('meg' == 'MEG')");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1m", parseOk), 1.0e-3) && parseOk,
                      "suffix matching is case-insensitive ('m' == 'M')");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1T", parseOk), 1.0e12) && parseOk, "suffix T = 1e12");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1G", parseOk), 1.0e9) && parseOk, "suffix G = 1e9");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1K", parseOk), 1.0e3) && parseOk, "suffix K = 1e3");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1U", parseOk), 1.0e-6) && parseOk, "suffix U = 1e-6");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1N", parseOk), 1.0e-9) && parseOk, "suffix N = 1e-9");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1P", parseOk), 1.0e-12) && parseOk, "suffix P = 1e-12");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1F", parseOk), 1.0e-15) && parseOk, "suffix F = 1e-15");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("4.7k", parseOk), 4700.0) && parseOk,
                      "decimal mantissa with suffix (4.7k == 4700)");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("1kOhm", parseOk), 1000.0) && parseOk,
                      "trailing unit text after suffix is ignored (1kOhm == 1k)");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("10uF", parseOk), 10.0e-6) && parseOk,
                      "trailing unit letter after suffix is ignored, not treated as a second suffix (10uF == 10u)");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("2.2e-9", parseOk), 2.2e-9) && parseOk,
                      "bare scientific notation with no suffix");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("-4.7k", parseOk), -4700.0) && parseOk,
                      "negative value with suffix");
        ok &= require(nearlyEqual(circuit::spice_detail::parseSpiceNumber("22", parseOk), 22.0) && parseOk,
                      "bare integer with no suffix");
        circuit::spice_detail::parseSpiceNumber("abc", parseOk);
        ok &= require(!parseOk, "non-numeric token is rejected");
    }

    // --- Ground aliasing ---
    {
        ok &= require(circuit::spice_detail::canonicalNode("0") == "0", "node '0' canonicalizes to ground");
        ok &= require(circuit::spice_detail::canonicalNode("GND") == "0", "node 'GND' canonicalizes to ground");
        ok &= require(circuit::spice_detail::canonicalNode("gnd") == "0",
                      "ground aliasing is case-insensitive ('gnd' == 'GND')");
        ok &= require(circuit::spice_detail::canonicalNode("N1") == circuit::spice_detail::canonicalNode("n1"),
                      "non-ground node names are canonicalized case-insensitively");
    }

    // --- Comments and continuation lines ---
    {
        const std::string text =
            "* full-line comment, ignored entirely\n"
            "R1 IN OUT 1k ; inline comment stripped\n"
            "C1 OUT 0\n"
            "+ 100n\n";
        const circuit::SpiceNetlist netlist = circuit::parseSpiceNetlist(text);
        ok &= require(netlist.elements.size() == 2, "full-line comment produces no element");
        ok &= require(netlist.elements[0].kind == 'R' && nearlyEqual(netlist.elements[0].values[0], 1000.0),
                      "inline ';' comment is stripped before parsing");
        ok &= require(netlist.elements[1].kind == 'C' && netlist.elements[1].nodes.size() == 2 &&
                      nearlyEqual(netlist.elements[1].values[0], 100.0e-9),
                      "'+' continuation line is joined onto the preceding card");
    }

    // --- Case-insensitivity of element kind and directive keywords ---
    {
        const circuit::SpiceNetlist netlist = circuit::parseSpiceNetlist("r1 in out 1k\n");
        ok &= require(netlist.elements.size() == 1 && netlist.elements[0].kind == 'R',
                      "lowercase element kind letter is accepted and canonicalized to uppercase");
    }

    // --- Element cards: one of each supported kind ---
    {
        const std::string text =
            "R1 N1 N2 4.7k\n"
            "L1 N2 N3 10m\n"
            "C1 N3 0 22n\n"
            "D1 N4 N5 D1N4148\n"
            "Q1 N6 N7 N8 Q2N3904\n"
            "V1 N9 0 DC 9\n"
            "I1 N10 0 1m\n"
            "E1 N11 0 N1 N2 1000\n";
        const circuit::SpiceNetlist netlist = circuit::parseSpiceNetlist(text);
        ok &= require(netlist.elements.size() == 8, "one card of each supported kind parses");

        const auto& r = netlist.elements[0];
        ok &= require(r.kind == 'R' && r.nodes == std::vector<std::string>{"N1", "N2"} &&
                      nearlyEqual(r.values[0], 4700.0), "resistor card: nodes and value");

        const auto& l = netlist.elements[1];
        ok &= require(l.kind == 'L' && nearlyEqual(l.values[0], 10.0e-3), "inductor card: value");

        const auto& c = netlist.elements[2];
        ok &= require(c.kind == 'C' && nearlyEqual(c.values[0], 22.0e-9), "capacitor card: value");

        const auto& d = netlist.elements[3];
        ok &= require(d.kind == 'D' && d.nodes.size() == 2 && d.modelName == "D1N4148",
                      "diode card: 2 nodes and model reference");

        const auto& q = netlist.elements[4];
        ok &= require(q.kind == 'Q' && q.nodes.size() == 3 && q.modelName == "Q2N3904",
                      "transistor card: collector/base/emitter and model reference");

        const auto& v = netlist.elements[5];
        ok &= require(v.kind == 'V' && nearlyEqual(v.values[0], 9.0), "voltage source card with 'DC' keyword");

        const auto& i = netlist.elements[6];
        ok &= require(i.kind == 'I' && nearlyEqual(i.values[0], 1.0e-3), "current source card without 'DC' keyword");

        const auto& e = netlist.elements[7];
        ok &= require(e.kind == 'E' && e.nodes.size() == 4 && nearlyEqual(e.values[0], 1000.0),
                      "VCVS card: 4 nodes and gain");
    }

    // --- Optional 4th (substrate) node on a transistor card ---
    {
        const circuit::SpiceNetlist netlist = circuit::parseSpiceNetlist("Q1 NC NB NE NS Q2N3904\n");
        ok &= require(netlist.elements.size() == 1 && netlist.elements[0].nodes.size() == 4,
                      "transistor card accepts an optional substrate node");
    }

    // --- .MODEL cards ---
    {
        const std::string text =
            ".model D1N4148 D(IS=2.52n RS=0.568 N=1.752)\n"
            ".MODEL Q2N3904 NPN(IS=6.734f BF=416.4 VAF=74.03)\n";
        const circuit::SpiceNetlist netlist = circuit::parseSpiceNetlist(text);
        ok &= require(netlist.models.size() == 2, "two .model cards parse");

        const auto diodeModel = netlist.models.find("D1N4148");
        ok &= require(diodeModel != netlist.models.end() && diodeModel->second.type == "D",
                      "diode .model type is 'D'");
        ok &= require(diodeModel != netlist.models.end() &&
                      nearlyEqual(diodeModel->second.params.at("IS"), 2.52e-9) &&
                      nearlyEqual(diodeModel->second.params.at("RS"), 0.568) &&
                      nearlyEqual(diodeModel->second.params.at("N"), 1.752),
                      "diode .model IS/RS/N parameters parse with correct suffixes");

        const auto bjtModel = netlist.models.find("Q2N3904");
        ok &= require(bjtModel != netlist.models.end() && bjtModel->second.type == "NPN",
                      "BJT .model type is 'NPN'");
        ok &= require(bjtModel != netlist.models.end() &&
                      nearlyEqual(bjtModel->second.params.at("IS"), 6.734e-15) &&
                      nearlyEqual(bjtModel->second.params.at("BF"), 416.4),
                      "BJT .model IS/BF parameters parse with correct suffixes");
    }

    // --- .MODEL is case-insensitive for the directive, name and type ---
    {
        const circuit::SpiceNetlist netlist = circuit::parseSpiceNetlist(".model d1 d(is=1n)\n");
        ok &= require(netlist.models.count("D1") == 1 && netlist.models.at("D1").type == "D",
                      ".model directive/name/type are canonicalized case-insensitively");
    }

    // --- Error reporting: line number and line content, per issue #99 ---
    {
        const std::string text = "R1 IN OUT 1k\nR2 OUT notanumber\n";
        const std::string message = expectError(text);
        ok &= require(message.find("line 2") != std::string::npos,
                      "error message includes the 1-based source line number");
        ok &= require(message.find("R2 OUT notanumber") != std::string::npos,
                      "error message includes the offending line's content");
    }
    {
        const std::string message = expectError("R1 IN OUT\n");
        ok &= require(!message.empty(), "resistor card missing a value is rejected");
    }
    {
        const std::string message = expectError("J1 D G S JFETMODEL\n");
        ok &= require(message.find("J") != std::string::npos, "excluded element type 'J' is rejected with a clear message");
    }
    {
        const std::string message = expectError(".SUBCKT FOO 1 2\n.ENDS\n");
        ok &= require(!message.empty(), "unsupported directive '.SUBCKT' is rejected, not silently ignored");
    }
    {
        const std::string message = expectError(".model X BOGUS(IS=1n)\n");
        ok &= require(!message.empty(), "unsupported .model type is rejected");
    }
    {
        const std::string message = expectError(".model D1 D(IS)\n");
        ok &= require(!message.empty(), ".model parameter without '=value' is rejected");
    }
    {
        const std::string message = expectError(".model D1 D(IS=1n)\n.model D1 D(IS=2n)\n");
        ok &= require(!message.empty(), "duplicate .model name is rejected");
    }
    {
        const std::string message = expectError("+ 1k\n");
        ok &= require(!message.empty(), "continuation line with no preceding card is rejected");
    }
    {
        const std::string message = expectError("R1 IN* OUT 1k\n");
        ok &= require(!message.empty(), "non-alphanumeric node name is rejected");
    }

    // --- Non-throwing convenience wrapper ---
    {
        circuit::SpiceNetlist netlist;
        std::string error;
        ok &= require(circuit::parseSpiceNetlist("R1 IN OUT 1k\n", netlist, &error) && error.empty(),
                      "bool-returning wrapper succeeds and clears error on valid input");
        ok &= require(!circuit::parseSpiceNetlist("R1 IN OUT bogus\n", netlist, &error) && !error.empty(),
                      "bool-returning wrapper fails and populates error on invalid input");
    }

    return ok ? 0 : 1;
}
