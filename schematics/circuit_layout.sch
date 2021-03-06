<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="6" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="74xx-us">
<description>&lt;b&gt;TTL Devices, 74xx Series with US Symbols&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Texas Instruments &lt;i&gt;TTL Data Book&lt;/i&gt;&amp;nbsp;&amp;nbsp;&amp;nbsp;Volume 1, 1996.
&lt;li&gt;TTL Data Book, Volume 2 , 1993
&lt;li&gt;National Seminconductor Databook 1990, ALS/LS Logic
&lt;li&gt;ttl 74er digital data dictionary, ECA Electronic + Acustic GmbH, ISBN 3-88109-032-0
&lt;li&gt;http://icmaster.com/ViewCompare.asp
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DIL16">
<description>&lt;b&gt;Dual In Line Package&lt;/b&gt;</description>
<wire x1="10.16" y1="2.921" x2="-10.16" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-2.921" x2="10.16" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="10.16" y1="2.921" x2="10.16" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="2.921" x2="-10.16" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-2.921" x2="-10.16" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="1.016" x2="-10.16" y2="-1.016" width="0.1524" layer="21" curve="-180"/>
<pad name="1" x="-8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="-1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="-1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="-3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="15" x="-6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="16" x="-8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-10.541" y="-2.921" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-7.493" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SO16">
<description>&lt;b&gt;Small Outline package&lt;/b&gt; 150 mil</description>
<wire x1="4.699" y1="1.9558" x2="-4.699" y2="1.9558" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.9558" x2="5.08" y2="-1.5748" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.5748" x2="-4.699" y2="1.9558" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.9558" x2="5.08" y2="1.5748" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.5748" x2="-4.699" y2="-1.9558" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="-1.9558" x2="4.699" y2="-1.9558" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.5748" x2="5.08" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.5748" x2="-5.08" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.508" x2="-5.08" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.508" x2="-5.08" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.508" x2="-5.08" y2="-0.508" width="0.1524" layer="21" curve="-180"/>
<wire x1="-5.08" y1="-1.6002" x2="5.08" y2="-1.6002" width="0.0508" layer="21"/>
<smd name="1" x="-4.445" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="16" x="-4.445" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="2" x="-3.175" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="3" x="-1.905" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="15" x="-3.175" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="14" x="-1.905" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="4" x="-0.635" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="13" x="-0.635" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="5" x="0.635" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="12" x="0.635" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="6" x="1.905" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="7" x="3.175" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="11" x="1.905" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="10" x="3.175" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="8" x="4.445" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="9" x="4.445" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<text x="-4.064" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-5.461" y="-1.778" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<rectangle x1="-0.889" y1="1.9558" x2="-0.381" y2="3.0988" layer="51"/>
<rectangle x1="-4.699" y1="-3.0988" x2="-4.191" y2="-1.9558" layer="51"/>
<rectangle x1="-3.429" y1="-3.0988" x2="-2.921" y2="-1.9558" layer="51"/>
<rectangle x1="-2.159" y1="-3.0734" x2="-1.651" y2="-1.9304" layer="51"/>
<rectangle x1="-0.889" y1="-3.0988" x2="-0.381" y2="-1.9558" layer="51"/>
<rectangle x1="-2.159" y1="1.9558" x2="-1.651" y2="3.0988" layer="51"/>
<rectangle x1="-3.429" y1="1.9558" x2="-2.921" y2="3.0988" layer="51"/>
<rectangle x1="-4.699" y1="1.9558" x2="-4.191" y2="3.0988" layer="51"/>
<rectangle x1="0.381" y1="-3.0988" x2="0.889" y2="-1.9558" layer="51"/>
<rectangle x1="1.651" y1="-3.0988" x2="2.159" y2="-1.9558" layer="51"/>
<rectangle x1="2.921" y1="-3.0988" x2="3.429" y2="-1.9558" layer="51"/>
<rectangle x1="4.191" y1="-3.0988" x2="4.699" y2="-1.9558" layer="51"/>
<rectangle x1="0.381" y1="1.9558" x2="0.889" y2="3.0988" layer="51"/>
<rectangle x1="1.651" y1="1.9558" x2="2.159" y2="3.0988" layer="51"/>
<rectangle x1="2.921" y1="1.9558" x2="3.429" y2="3.0988" layer="51"/>
<rectangle x1="4.191" y1="1.9558" x2="4.699" y2="3.0988" layer="51"/>
</package>
<package name="LCC20">
<description>&lt;b&gt;Leadless Chip Carrier&lt;/b&gt;&lt;p&gt; Ceramic Package</description>
<wire x1="-0.4001" y1="4.4" x2="-0.87" y2="4.4" width="0.2032" layer="51"/>
<wire x1="-3.3" y1="4.4" x2="-4.4" y2="3.3" width="0.2032" layer="51"/>
<wire x1="-0.4001" y1="4.3985" x2="0.4001" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-1.6701" y1="4.3985" x2="-0.8699" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="2.14" x2="-4.3985" y2="2.94" width="0.2032" layer="51" curve="180"/>
<wire x1="-2.9401" y1="4.4" x2="-3.3" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.87" y1="4.4" x2="0.4001" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.87" y1="4.3985" x2="1.67" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="3.3" x2="-4.4" y2="2.9401" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="2.14" x2="-4.4" y2="1.6701" width="0.2032" layer="51"/>
<wire x1="-4.3985" y1="0.87" x2="-4.3985" y2="1.67" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="-0.4001" x2="-4.3985" y2="0.4001" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="-1.6701" x2="-4.3985" y2="-0.8699" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="0.87" x2="-4.4" y2="0.4001" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-0.4001" x2="-4.4" y2="-0.87" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-2.9401" x2="-4.4" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-4.4" x2="-4.4" y2="-4.4099" width="0.2032" layer="51"/>
<wire x1="2.14" y1="4.3985" x2="2.94" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="2.14" y1="4.4" x2="1.6701" y2="4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="4.4" x2="2.9401" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.4001" y1="-4.4" x2="0.87" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-0.4001" y1="-4.3985" x2="0.4001" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="0.87" y1="-4.3985" x2="1.67" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="2.9401" y1="-4.4" x2="4.4" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-0.87" y1="-4.4" x2="-0.4001" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-1.6701" y1="-4.3985" x2="-0.8699" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="-2.9401" y1="-4.3985" x2="-2.1399" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="-2.14" y1="-4.4" x2="-1.6701" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-4.4" x2="-2.9401" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="0.4001" x2="4.4" y2="0.87" width="0.2032" layer="51"/>
<wire x1="4.3985" y1="0.4001" x2="4.3985" y2="-0.4001" width="0.2032" layer="51" curve="180"/>
<wire x1="4.3985" y1="1.6701" x2="4.3985" y2="0.8699" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="2.9401" x2="4.4" y2="4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="-0.87" x2="4.4" y2="-0.4001" width="0.2032" layer="51"/>
<wire x1="4.3985" y1="-0.87" x2="4.3985" y2="-1.67" width="0.2032" layer="51" curve="180"/>
<wire x1="4.3985" y1="-2.14" x2="4.3985" y2="-2.94" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="-2.14" x2="4.4" y2="-1.6701" width="0.2032" layer="51"/>
<wire x1="4.4" y1="-4.4" x2="4.4" y2="-2.9401" width="0.2032" layer="51"/>
<wire x1="-2.9401" y1="4.3985" x2="-2.1399" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-1.6701" y1="4.4" x2="-2.14" y2="4.4" width="0.2032" layer="51"/>
<wire x1="-4.3985" y1="-2.9401" x2="-4.3985" y2="-2.1399" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="-1.6701" x2="-4.4" y2="-2.14" width="0.2032" layer="51"/>
<wire x1="1.6701" y1="-4.4" x2="2.14" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="2.14" y1="-4.3985" x2="2.94" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="4.3985" y1="2.9401" x2="4.3985" y2="2.1399" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="1.6701" x2="4.4" y2="2.14" width="0.2032" layer="51"/>
<smd name="2" x="-1.27" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="1" x="0" y="3.8001" dx="0.8" dy="3.4" layer="1"/>
<smd name="3" x="-2.54" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="4" x="-4.5001" y="2.54" dx="2" dy="0.8" layer="1"/>
<smd name="5" x="-4.5001" y="1.27" dx="2" dy="0.8" layer="1"/>
<smd name="6" x="-4.5001" y="0" dx="2" dy="0.8" layer="1"/>
<smd name="7" x="-4.5001" y="-1.27" dx="2" dy="0.8" layer="1"/>
<smd name="8" x="-4.5001" y="-2.54" dx="2" dy="0.8" layer="1"/>
<smd name="9" x="-2.54" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="10" x="-1.27" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="11" x="0" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="12" x="1.27" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="13" x="2.54" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="14" x="4.5001" y="-2.54" dx="2" dy="0.8" layer="1"/>
<smd name="15" x="4.5001" y="-1.27" dx="2" dy="0.8" layer="1"/>
<smd name="16" x="4.5001" y="0" dx="2" dy="0.8" layer="1"/>
<smd name="17" x="4.5001" y="1.27" dx="2" dy="0.8" layer="1"/>
<smd name="18" x="4.5001" y="2.54" dx="2" dy="0.8" layer="1"/>
<smd name="19" x="2.54" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="20" x="1.27" y="4.5001" dx="0.8" dy="2" layer="1"/>
<text x="-3.4971" y="5.811" size="1.778" layer="25">&gt;NAME</text>
<text x="-3.9751" y="-7.6871" size="1.778" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="74165">
<wire x1="-7.62" y1="-12.7" x2="7.62" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-12.7" x2="7.62" y2="10.16" width="0.4064" layer="94"/>
<wire x1="7.62" y1="10.16" x2="-7.62" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="10.16" x2="-7.62" y2="-12.7" width="0.4064" layer="94"/>
<text x="-7.62" y="10.795" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="-15.24" size="1.778" layer="96">&gt;VALUE</text>
<pin name="PL" x="-12.7" y="7.62" length="middle" direction="in" function="dot"/>
<pin name="CP" x="-12.7" y="5.08" length="middle" direction="in" function="clk"/>
<pin name="D4" x="-12.7" y="2.54" length="middle" direction="in"/>
<pin name="D5" x="-12.7" y="0" length="middle" direction="in"/>
<pin name="D6" x="-12.7" y="-2.54" length="middle" direction="in"/>
<pin name="D7" x="-12.7" y="-5.08" length="middle" direction="in"/>
<pin name="!Q7" x="-12.7" y="-7.62" length="middle" direction="out"/>
<pin name="Q7" x="12.7" y="-10.16" length="middle" direction="out" rot="R180"/>
<pin name="DS" x="12.7" y="-7.62" length="middle" direction="in" rot="R180"/>
<pin name="D0" x="12.7" y="-5.08" length="middle" direction="in" rot="R180"/>
<pin name="D1" x="12.7" y="-2.54" length="middle" direction="in" rot="R180"/>
<pin name="D2" x="12.7" y="0" length="middle" direction="in" rot="R180"/>
<pin name="D3" x="12.7" y="2.54" length="middle" direction="in" rot="R180"/>
<pin name="CE" x="12.7" y="5.08" length="middle" direction="in" function="dot" rot="R180"/>
</symbol>
<symbol name="PWRN">
<text x="-0.635" y="-0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.905" y="-7.62" size="1.27" layer="95" rot="R90">GND</text>
<text x="1.905" y="5.08" size="1.27" layer="95" rot="R90">VCC</text>
<pin name="GND" x="0" y="-10.16" visible="pad" direction="pwr" rot="R90"/>
<pin name="VCC" x="0" y="10.16" visible="pad" direction="pwr" rot="R270"/>
</symbol>
<symbol name="74595">
<wire x1="-7.62" y1="-12.7" x2="7.62" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-12.7" x2="7.62" y2="10.16" width="0.4064" layer="94"/>
<wire x1="7.62" y1="10.16" x2="-7.62" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="10.16" x2="-7.62" y2="-12.7" width="0.4064" layer="94"/>
<text x="-7.62" y="10.795" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="-15.24" size="1.778" layer="96">&gt;VALUE</text>
<pin name="Q1" x="-12.7" y="7.62" length="middle" direction="hiz"/>
<pin name="Q2" x="-12.7" y="5.08" length="middle" direction="hiz"/>
<pin name="Q3" x="-12.7" y="2.54" length="middle" direction="hiz"/>
<pin name="Q4" x="-12.7" y="0" length="middle" direction="hiz"/>
<pin name="Q5" x="-12.7" y="-2.54" length="middle" direction="hiz"/>
<pin name="Q6" x="-12.7" y="-5.08" length="middle" direction="hiz"/>
<pin name="Q7" x="-12.7" y="-7.62" length="middle" direction="hiz"/>
<pin name="MR" x="12.7" y="-7.62" length="middle" direction="in" function="dot" rot="R180"/>
<pin name="SHCP" x="12.7" y="-5.08" length="middle" direction="in" function="clk" rot="R180"/>
<pin name="STCP" x="12.7" y="-2.54" length="middle" direction="in" function="clk" rot="R180"/>
<pin name="OE" x="12.7" y="0" length="middle" direction="in" function="dot" rot="R180"/>
<pin name="DS" x="12.7" y="2.54" length="middle" direction="in" rot="R180"/>
<pin name="Q0" x="12.7" y="5.08" length="middle" direction="hiz" rot="R180"/>
<pin name="QH*" x="12.7" y="-10.16" length="middle" direction="hiz" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="74*165" prefix="IC">
<description>8-bit parallel load &lt;b&gt;SHIFT REGISTER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="74165" x="20.32" y="0"/>
<gate name="P" symbol="PWRN" x="-5.08" y="0" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL16">
<connects>
<connect gate="A" pin="!Q7" pad="7"/>
<connect gate="A" pin="CE" pad="15"/>
<connect gate="A" pin="CP" pad="2"/>
<connect gate="A" pin="D0" pad="11"/>
<connect gate="A" pin="D1" pad="12"/>
<connect gate="A" pin="D2" pad="13"/>
<connect gate="A" pin="D3" pad="14"/>
<connect gate="A" pin="D4" pad="3"/>
<connect gate="A" pin="D5" pad="4"/>
<connect gate="A" pin="D6" pad="5"/>
<connect gate="A" pin="D7" pad="6"/>
<connect gate="A" pin="DS" pad="10"/>
<connect gate="A" pin="PL" pad="1"/>
<connect gate="A" pin="Q7" pad="9"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name=""/>
<technology name="LS"/>
</technologies>
</device>
<device name="D" package="SO16">
<connects>
<connect gate="A" pin="!Q7" pad="7"/>
<connect gate="A" pin="CE" pad="15"/>
<connect gate="A" pin="CP" pad="2"/>
<connect gate="A" pin="D0" pad="11"/>
<connect gate="A" pin="D1" pad="12"/>
<connect gate="A" pin="D2" pad="13"/>
<connect gate="A" pin="D3" pad="14"/>
<connect gate="A" pin="D4" pad="3"/>
<connect gate="A" pin="D5" pad="4"/>
<connect gate="A" pin="D6" pad="5"/>
<connect gate="A" pin="D7" pad="6"/>
<connect gate="A" pin="DS" pad="10"/>
<connect gate="A" pin="PL" pad="1"/>
<connect gate="A" pin="Q7" pad="9"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name="LS"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="!Q7" pad="9"/>
<connect gate="A" pin="CE" pad="19"/>
<connect gate="A" pin="CP" pad="3"/>
<connect gate="A" pin="D0" pad="14"/>
<connect gate="A" pin="D1" pad="15"/>
<connect gate="A" pin="D2" pad="17"/>
<connect gate="A" pin="D3" pad="18"/>
<connect gate="A" pin="D4" pad="4"/>
<connect gate="A" pin="D5" pad="5"/>
<connect gate="A" pin="D6" pad="7"/>
<connect gate="A" pin="D7" pad="8"/>
<connect gate="A" pin="DS" pad="13"/>
<connect gate="A" pin="PL" pad="2"/>
<connect gate="A" pin="Q7" pad="12"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<technologies>
<technology name="LS"/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="74*595" prefix="IC">
<description>8-bit &lt;b&gt;SHIFT REGISTER&lt;/b&gt;, output latch</description>
<gates>
<gate name="A" symbol="74595" x="22.86" y="0"/>
<gate name="P" symbol="PWRN" x="-5.08" y="0" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL16">
<connects>
<connect gate="A" pin="DS" pad="14"/>
<connect gate="A" pin="MR" pad="10"/>
<connect gate="A" pin="OE" pad="13"/>
<connect gate="A" pin="Q0" pad="15"/>
<connect gate="A" pin="Q1" pad="1"/>
<connect gate="A" pin="Q2" pad="2"/>
<connect gate="A" pin="Q3" pad="3"/>
<connect gate="A" pin="Q4" pad="4"/>
<connect gate="A" pin="Q5" pad="5"/>
<connect gate="A" pin="Q6" pad="6"/>
<connect gate="A" pin="Q7" pad="7"/>
<connect gate="A" pin="QH*" pad="9"/>
<connect gate="A" pin="SHCP" pad="11"/>
<connect gate="A" pin="STCP" pad="12"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name="LS"/>
</technologies>
</device>
<device name="D" package="SO16">
<connects>
<connect gate="A" pin="DS" pad="14"/>
<connect gate="A" pin="MR" pad="10"/>
<connect gate="A" pin="OE" pad="13"/>
<connect gate="A" pin="Q0" pad="15"/>
<connect gate="A" pin="Q1" pad="1"/>
<connect gate="A" pin="Q2" pad="2"/>
<connect gate="A" pin="Q3" pad="3"/>
<connect gate="A" pin="Q4" pad="4"/>
<connect gate="A" pin="Q5" pad="5"/>
<connect gate="A" pin="Q6" pad="6"/>
<connect gate="A" pin="Q7" pad="7"/>
<connect gate="A" pin="QH*" pad="9"/>
<connect gate="A" pin="SHCP" pad="11"/>
<connect gate="A" pin="STCP" pad="12"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name="LS"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="DS" pad="18"/>
<connect gate="A" pin="MR" pad="13"/>
<connect gate="A" pin="OE" pad="17"/>
<connect gate="A" pin="Q0" pad="19"/>
<connect gate="A" pin="Q1" pad="2"/>
<connect gate="A" pin="Q2" pad="3"/>
<connect gate="A" pin="Q3" pad="4"/>
<connect gate="A" pin="Q4" pad="5"/>
<connect gate="A" pin="Q5" pad="7"/>
<connect gate="A" pin="Q6" pad="8"/>
<connect gate="A" pin="Q7" pad="9"/>
<connect gate="A" pin="QH*" pad="12"/>
<connect gate="A" pin="SHCP" pad="14"/>
<connect gate="A" pin="STCP" pad="15"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<technologies>
<technology name="LS"/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="MSP430G2553">
<packages>
<package name="MSP430G2553">
<pad name="P$1" x="-3.81" y="3.81" drill="0.8" shape="square"/>
<pad name="P$2" x="-3.81" y="2.54" drill="0.8" shape="square"/>
<pad name="P$3" x="-3.81" y="1.27" drill="0.8" shape="square"/>
<pad name="P$4" x="-3.81" y="0" drill="0.8" shape="square"/>
<pad name="P$5" x="-3.81" y="-1.27" drill="0.8" shape="square"/>
<pad name="P$6" x="-3.81" y="-2.54" drill="0.8" shape="square"/>
<pad name="P$7" x="-3.81" y="-3.81" drill="0.8" shape="square"/>
<pad name="P$8" x="-3.81" y="-5.08" drill="0.8" shape="square"/>
<pad name="P$9" x="3.81" y="0" drill="0.8" shape="square"/>
<pad name="P$10" x="3.81" y="1.27" drill="0.8" shape="square"/>
<pad name="P$11" x="3.81" y="2.54" drill="0.8" shape="square"/>
<pad name="P$12" x="3.81" y="3.81" drill="0.8" shape="square"/>
<pad name="P$13" x="3.81" y="-1.27" drill="0.8" shape="square"/>
<pad name="P$14" x="3.81" y="-2.54" drill="0.8" shape="square"/>
<pad name="P$15" x="3.81" y="-3.81" drill="0.8" shape="square"/>
<pad name="P$16" x="3.81" y="-5.08" drill="0.8" shape="square"/>
</package>
</packages>
<symbols>
<symbol name="MSP430G2553">
<wire x1="-7.62" y1="12.7" x2="-7.62" y2="-15.24" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-15.24" x2="10.16" y2="-15.24" width="0.254" layer="94"/>
<wire x1="10.16" y1="-15.24" x2="10.16" y2="12.7" width="0.254" layer="94"/>
<wire x1="10.16" y1="12.7" x2="-7.62" y2="12.7" width="0.254" layer="94"/>
<pin name="P1.0" x="-12.7" y="7.62" visible="pin" length="middle"/>
<pin name="P1.1" x="-12.7" y="5.08" visible="pin" length="middle"/>
<pin name="P1.2" x="-12.7" y="2.54" visible="pin" length="middle"/>
<pin name="P1.3" x="-12.7" y="0" visible="pin" length="middle"/>
<pin name="P1.4" x="-12.7" y="-2.54" visible="pin" length="middle"/>
<pin name="P1.5" x="-12.7" y="-5.08" visible="pin" length="middle"/>
<pin name="P1.6" x="15.24" y="-5.08" visible="pin" length="middle" rot="R180"/>
<pin name="P1.7" x="15.24" y="-2.54" visible="pin" length="middle" rot="R180"/>
<pin name="P2.0" x="-12.7" y="-7.62" visible="pin" length="middle"/>
<pin name="P2.1" x="-12.7" y="-10.16" visible="pin" length="middle"/>
<pin name="P2.2" x="-12.7" y="-12.7" visible="pin" length="middle"/>
<pin name="P2.3" x="15.24" y="-12.7" visible="pin" length="middle" rot="R180"/>
<pin name="P2.4" x="15.24" y="-10.16" visible="pin" length="middle" rot="R180"/>
<pin name="P2.5" x="15.24" y="-7.62" visible="pin" length="middle" rot="R180"/>
<pin name="P2.6" x="15.24" y="7.62" visible="pin" length="middle" rot="R180"/>
<pin name="P2.7" x="15.24" y="5.08" visible="pin" length="middle" rot="R180"/>
<text x="-7.62" y="13.716" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="-17.78" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="MSP430G2553">
<gates>
<gate name="G$1" symbol="MSP430G2553" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MSP430G2553">
<connects>
<connect gate="G$1" pin="P1.0" pad="P$1"/>
<connect gate="G$1" pin="P1.1" pad="P$2"/>
<connect gate="G$1" pin="P1.2" pad="P$3"/>
<connect gate="G$1" pin="P1.3" pad="P$4"/>
<connect gate="G$1" pin="P1.4" pad="P$5"/>
<connect gate="G$1" pin="P1.5" pad="P$6"/>
<connect gate="G$1" pin="P1.6" pad="P$7"/>
<connect gate="G$1" pin="P1.7" pad="P$8"/>
<connect gate="G$1" pin="P2.0" pad="P$9"/>
<connect gate="G$1" pin="P2.1" pad="P$10"/>
<connect gate="G$1" pin="P2.2" pad="P$11"/>
<connect gate="G$1" pin="P2.3" pad="P$12"/>
<connect gate="G$1" pin="P2.4" pad="P$13"/>
<connect gate="G$1" pin="P2.5" pad="P$14"/>
<connect gate="G$1" pin="P2.6" pad="P$15"/>
<connect gate="G$1" pin="P2.7" pad="P$16"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="AS6C1008">
<packages>
<package name="AS6C1008">
<pad name="P$1" x="-1.27" y="3.81" drill="0.8" shape="square"/>
<pad name="P$2" x="-1.27" y="2.54" drill="0.8" shape="square"/>
<pad name="P$3" x="-1.27" y="1.27" drill="0.8" shape="square"/>
<pad name="P$4" x="-1.27" y="0" drill="0.8" shape="square"/>
<pad name="P$5" x="-1.27" y="-1.27" drill="0.8" shape="square"/>
<pad name="P$6" x="-1.27" y="-2.54" drill="0.8" shape="square"/>
<pad name="P$7" x="-1.27" y="-3.81" drill="0.8" shape="square"/>
<pad name="P$8" x="-1.27" y="-5.08" drill="0.8" shape="square"/>
<pad name="P$9" x="-1.27" y="-6.35" drill="0.8" shape="square"/>
<pad name="P$10" x="-1.27" y="-7.62" drill="0.8" shape="square"/>
<pad name="P$11" x="-1.27" y="-8.89" drill="0.8" shape="square"/>
<pad name="P$12" x="-1.27" y="-10.16" drill="0.8" shape="square"/>
<pad name="P$13" x="-1.27" y="-11.43" drill="0.8" shape="square"/>
<pad name="P$14" x="-1.27" y="-12.7" drill="0.8" shape="square"/>
<pad name="P$15" x="-1.27" y="-13.97" drill="0.8" shape="square"/>
<pad name="P$16" x="-1.27" y="-15.24" drill="0.8" shape="square"/>
<pad name="P$17" x="6.35" y="-15.24" drill="0.8" shape="square"/>
<pad name="P$18" x="6.35" y="-13.97" drill="0.8" shape="square"/>
<pad name="P$19" x="6.35" y="-12.7" drill="0.8" shape="square"/>
<pad name="P$20" x="6.35" y="-11.43" drill="0.8" shape="square"/>
<pad name="P$21" x="6.35" y="-10.16" drill="0.8" shape="square"/>
<pad name="P$22" x="6.35" y="-8.89" drill="0.8" shape="square"/>
<pad name="P$23" x="6.35" y="-7.62" drill="0.8" shape="square"/>
<pad name="P$24" x="6.35" y="-6.35" drill="0.8" shape="square"/>
<pad name="P$25" x="6.35" y="-5.08" drill="0.8" shape="square"/>
<pad name="P$26" x="6.35" y="-3.81" drill="0.8" shape="square"/>
<pad name="P$27" x="6.35" y="-2.54" drill="0.8" shape="square"/>
<pad name="P$28" x="6.35" y="-1.27" drill="0.8" shape="square"/>
<pad name="P$29" x="6.35" y="0" drill="0.8" shape="square"/>
<pad name="P$30" x="6.35" y="1.27" drill="0.8" shape="square"/>
<pad name="P$31" x="6.35" y="2.54" drill="0.8" shape="square"/>
<pad name="P$32" x="6.35" y="3.81" drill="0.8" shape="square"/>
</package>
</packages>
<symbols>
<symbol name="AS6C1008">
<pin name="A0" x="-12.7" y="-10.16" visible="pin" length="middle"/>
<pin name="A1" x="-12.7" y="-7.62" visible="pin" length="middle"/>
<pin name="A2" x="-12.7" y="-5.08" visible="pin" length="middle"/>
<pin name="A3" x="-12.7" y="-2.54" visible="pin" length="middle"/>
<pin name="A4" x="-12.7" y="0" visible="pin" length="middle"/>
<pin name="A5" x="-12.7" y="2.54" visible="pin" length="middle"/>
<pin name="A6" x="-12.7" y="5.08" visible="pin" length="middle"/>
<pin name="A7" x="-12.7" y="7.62" visible="pin" length="middle"/>
<pin name="A8" x="12.7" y="5.08" visible="pin" length="middle" rot="R180"/>
<pin name="A9" x="12.7" y="2.54" visible="pin" length="middle" rot="R180"/>
<pin name="A10" x="12.7" y="-5.08" visible="pin" length="middle" rot="R180"/>
<pin name="A11" x="12.7" y="0" visible="pin" length="middle" rot="R180"/>
<pin name="A12" x="-12.7" y="10.16" visible="pin" length="middle"/>
<pin name="A13" x="12.7" y="7.62" visible="pin" length="middle" rot="R180"/>
<pin name="A14" x="-12.7" y="12.7" visible="pin" length="middle"/>
<pin name="A15" x="12.7" y="15.24" visible="pin" length="middle" rot="R180"/>
<wire x1="-7.62" y1="20.32" x2="7.62" y2="20.32" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-22.86" x2="-7.62" y2="20.32" width="0.254" layer="94"/>
<wire x1="7.62" y1="20.32" x2="7.62" y2="-22.86" width="0.254" layer="94"/>
<wire x1="7.62" y1="-22.86" x2="-7.62" y2="-22.86" width="0.254" layer="94"/>
<pin name="DQ0" x="-12.7" y="-12.7" visible="pin" length="middle"/>
<pin name="DQ1" x="-12.7" y="-15.24" visible="pin" length="middle"/>
<pin name="DQ2" x="-12.7" y="-17.78" visible="pin" length="middle"/>
<pin name="DQ3" x="12.7" y="-20.32" visible="pin" length="middle" rot="R180"/>
<pin name="DQ4" x="12.7" y="-17.78" visible="pin" length="middle" rot="R180"/>
<pin name="DQ5" x="12.7" y="-15.24" visible="pin" length="middle" rot="R180"/>
<pin name="DQ6" x="12.7" y="-12.7" visible="pin" length="middle" rot="R180"/>
<pin name="DQ7" x="12.7" y="-10.16" visible="pin" length="middle" rot="R180"/>
<pin name="CE" x="12.7" y="-7.62" visible="pin" length="middle" function="dot" rot="R180"/>
<pin name="CE2" x="12.7" y="12.7" visible="pin" length="middle" rot="R180"/>
<pin name="A16" x="-12.7" y="15.24" visible="pin" length="middle"/>
<pin name="WE" x="12.7" y="10.16" visible="pin" length="middle" function="dot" rot="R180"/>
<pin name="OE" x="12.7" y="-2.54" visible="pin" length="middle" function="dot" rot="R180"/>
<text x="-7.62" y="-25.4" size="1.778" layer="96">&gt;VALUE</text>
<text x="-7.62" y="21.082" size="1.778" layer="95">&gt;NAME</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="AS6C1008">
<gates>
<gate name="G$1" symbol="AS6C1008" x="0" y="0"/>
</gates>
<devices>
<device name="" package="AS6C1008">
<connects>
<connect gate="G$1" pin="A0" pad="P$1"/>
<connect gate="G$1" pin="A1" pad="P$2"/>
<connect gate="G$1" pin="A10" pad="P$11"/>
<connect gate="G$1" pin="A11" pad="P$12"/>
<connect gate="G$1" pin="A12" pad="P$13"/>
<connect gate="G$1" pin="A13" pad="P$14"/>
<connect gate="G$1" pin="A14" pad="P$15"/>
<connect gate="G$1" pin="A15" pad="P$16"/>
<connect gate="G$1" pin="A16" pad="P$17"/>
<connect gate="G$1" pin="A2" pad="P$3"/>
<connect gate="G$1" pin="A3" pad="P$4"/>
<connect gate="G$1" pin="A4" pad="P$5"/>
<connect gate="G$1" pin="A5" pad="P$6"/>
<connect gate="G$1" pin="A6" pad="P$7"/>
<connect gate="G$1" pin="A7" pad="P$8"/>
<connect gate="G$1" pin="A8" pad="P$9"/>
<connect gate="G$1" pin="A9" pad="P$10"/>
<connect gate="G$1" pin="CE" pad="P$18"/>
<connect gate="G$1" pin="CE2" pad="P$19"/>
<connect gate="G$1" pin="DQ0" pad="P$20"/>
<connect gate="G$1" pin="DQ1" pad="P$21"/>
<connect gate="G$1" pin="DQ2" pad="P$22"/>
<connect gate="G$1" pin="DQ3" pad="P$23"/>
<connect gate="G$1" pin="DQ4" pad="P$24"/>
<connect gate="G$1" pin="DQ5" pad="P$25"/>
<connect gate="G$1" pin="DQ6" pad="P$26"/>
<connect gate="G$1" pin="DQ7" pad="P$27"/>
<connect gate="G$1" pin="OE" pad="P$28"/>
<connect gate="G$1" pin="WE" pad="P$29"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="IC1" library="74xx-us" deviceset="74*165" device="N"/>
<part name="U$1" library="MSP430G2553" deviceset="MSP430G2553" device=""/>
<part name="U$2" library="MSP430G2553" deviceset="MSP430G2553" device=""/>
<part name="U$3" library="AS6C1008" deviceset="AS6C1008" device=""/>
<part name="IC2" library="74xx-us" deviceset="74*595" device="D" technology="LS"/>
<part name="IC3" library="74xx-us" deviceset="74*595" device="D" technology="LS"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="IC1" gate="A" x="-25.4" y="0" rot="R180"/>
<instance part="U$1" gate="G$1" x="7.62" y="40.64" rot="R180"/>
<instance part="U$2" gate="G$1" x="53.34" y="40.64"/>
<instance part="U$3" gate="G$1" x="55.88" y="-5.08" rot="R180"/>
<instance part="IC2" gate="A" x="17.78" y="-10.16" rot="R180"/>
<instance part="IC3" gate="A" x="93.98" y="-12.7"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q7"/>
<pinref part="U$3" gate="G$1" pin="A16"/>
<wire x1="81.28" y1="-20.32" x2="68.58" y2="-20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q6"/>
<pinref part="U$3" gate="G$1" pin="A14"/>
<wire x1="81.28" y1="-17.78" x2="68.58" y2="-17.78" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q5"/>
<pinref part="U$3" gate="G$1" pin="A12"/>
<wire x1="81.28" y1="-15.24" x2="68.58" y2="-15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q4"/>
<pinref part="U$3" gate="G$1" pin="A7"/>
<wire x1="81.28" y1="-12.7" x2="68.58" y2="-12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q3"/>
<pinref part="U$3" gate="G$1" pin="A6"/>
<wire x1="81.28" y1="-10.16" x2="68.58" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q2"/>
<pinref part="U$3" gate="G$1" pin="A5"/>
<wire x1="81.28" y1="-7.62" x2="68.58" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q1"/>
<pinref part="U$3" gate="G$1" pin="A4"/>
<wire x1="81.28" y1="-5.08" x2="68.58" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="IC3" gate="A" pin="Q0"/>
<pinref part="U$3" gate="G$1" pin="A3"/>
<wire x1="106.68" y1="-7.62" x2="68.58" y2="-2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q1"/>
<pinref part="U$3" gate="G$1" pin="A15"/>
<wire x1="30.48" y1="-17.78" x2="43.18" y2="-20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q2"/>
<pinref part="U$3" gate="G$1" pin="A13"/>
<wire x1="30.48" y1="-15.24" x2="43.18" y2="-12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q3"/>
<pinref part="U$3" gate="G$1" pin="A8"/>
<wire x1="30.48" y1="-12.7" x2="43.18" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q4"/>
<pinref part="U$3" gate="G$1" pin="A9"/>
<wire x1="30.48" y1="-10.16" x2="43.18" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q5"/>
<pinref part="U$3" gate="G$1" pin="A11"/>
<wire x1="30.48" y1="-7.62" x2="43.18" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q6"/>
<pinref part="U$3" gate="G$1" pin="A10"/>
<wire x1="30.48" y1="-5.08" x2="43.18" y2="0" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q7"/>
<pinref part="U$3" gate="G$1" pin="A2"/>
<wire x1="30.48" y1="-2.54" x2="68.58" y2="0" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="IC2" gate="A" pin="Q0"/>
<pinref part="U$3" gate="G$1" pin="A1"/>
<wire x1="5.08" y1="-15.24" x2="68.58" y2="2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.6"/>
<wire x1="68.58" y1="48.26" x2="68.58" y2="55.88" width="0.1524" layer="91"/>
<wire x1="68.58" y1="55.88" x2="30.48" y2="55.88" width="0.1524" layer="91"/>
<wire x1="30.48" y1="55.88" x2="30.48" y2="7.62" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="DQ6"/>
<wire x1="30.48" y1="7.62" x2="43.18" y2="7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.7"/>
<wire x1="68.58" y1="45.72" x2="71.12" y2="45.72" width="0.1524" layer="91"/>
<wire x1="71.12" y1="45.72" x2="71.12" y2="58.42" width="0.1524" layer="91"/>
<wire x1="71.12" y1="58.42" x2="27.94" y2="58.42" width="0.1524" layer="91"/>
<wire x1="27.94" y1="58.42" x2="27.94" y2="5.08" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="DQ7"/>
<wire x1="27.94" y1="5.08" x2="43.18" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="IC3" gate="A" pin="DS"/>
<wire x1="106.68" y1="-10.16" x2="109.22" y2="-10.16" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="QH*"/>
<wire x1="5.08" y1="0" x2="109.22" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P1.5"/>
<pinref part="IC2" gate="A" pin="SHCP"/>
<wire x1="40.64" y1="35.56" x2="5.08" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="SHCP"/>
<wire x1="40.64" y1="35.56" x2="106.68" y2="-17.78" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P1.1"/>
<pinref part="U$1" gate="G$1" pin="P1.2"/>
<wire x1="40.64" y1="45.72" x2="20.32" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$30" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P1.2"/>
<pinref part="U$1" gate="G$1" pin="P1.1"/>
<wire x1="40.64" y1="43.18" x2="20.32" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$31" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P1.0"/>
<pinref part="IC2" gate="A" pin="STCP"/>
<wire x1="40.64" y1="48.26" x2="5.08" y2="-7.62" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="STCP"/>
<wire x1="40.64" y1="48.26" x2="106.68" y2="-15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$32" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P1.0"/>
<pinref part="U$3" gate="G$1" pin="A0"/>
<wire x1="20.32" y1="33.02" x2="68.58" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P1.3"/>
<pinref part="IC1" gate="A" pin="CP"/>
<wire x1="20.32" y1="40.64" x2="-12.7" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P1.4"/>
<pinref part="IC1" gate="A" pin="PL"/>
<wire x1="20.32" y1="43.18" x2="-12.7" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$35" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P1.5"/>
<pinref part="IC1" gate="A" pin="Q7"/>
<wire x1="20.32" y1="45.72" x2="-38.1" y2="10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.0"/>
<pinref part="U$3" gate="G$1" pin="DQ0"/>
<wire x1="40.64" y1="33.02" x2="68.58" y2="7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.1"/>
<pinref part="U$3" gate="G$1" pin="DQ1"/>
<wire x1="40.64" y1="30.48" x2="68.58" y2="10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.2"/>
<pinref part="U$3" gate="G$1" pin="DQ2"/>
<wire x1="40.64" y1="27.94" x2="68.58" y2="12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.3"/>
<pinref part="U$3" gate="G$1" pin="DQ3"/>
<wire x1="68.58" y1="27.94" x2="43.18" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.4"/>
<pinref part="U$3" gate="G$1" pin="DQ4"/>
<wire x1="68.58" y1="30.48" x2="43.18" y2="12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P2.5"/>
<pinref part="U$3" gate="G$1" pin="DQ5"/>
<wire x1="68.58" y1="33.02" x2="43.18" y2="10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$27" class="0">
<segment>
<pinref part="U$3" gate="G$1" pin="WE"/>
<pinref part="U$2" gate="G$1" pin="P1.4"/>
<wire x1="43.18" y1="-15.24" x2="40.64" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P1.3"/>
<wire x1="40.64" y1="40.64" x2="38.1" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="OE"/>
<wire x1="38.1" y1="40.64" x2="43.18" y2="-2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$36" class="0">
<segment>
<pinref part="IC2" gate="A" pin="DS"/>
<pinref part="U$2" gate="G$1" pin="P1.7"/>
<wire x1="68.58" y1="38.1" x2="5.08" y2="-12.7" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
