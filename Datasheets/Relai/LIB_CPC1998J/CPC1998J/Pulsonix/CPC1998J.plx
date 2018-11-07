PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//324398/24223/2.16/4/3/Relay or Contactor

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c249_h166"
		(holeDiam 1.66)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 2.49) (shapeHeight 2.49))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 2.49) (shapeHeight 2.49))
	)
	(padStyleDef "s249_h166"
		(holeDiam 1.66)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 2.49) (shapeHeight 2.49))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 2.49) (shapeHeight 2.49))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "TO381P516X1991X2388-4P" (originalName "TO381P516X1991X2388-4P")
		(multiLayer
			(pad (padNum 1) (padStyleRef s249_h166) (pt 0, 0) (rotation 90))
			(pad (padNum 2) (padStyleRef c249_h166) (pt 3.81, 0) (rotation 90))
			(pad (padNum 3) (padStyleRef c249_h166) (pt 7.62, 0) (rotation 90))
			(pad (padNum 4) (padStyleRef c249_h166) (pt 11.43, 0) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -4.619 3.171) (pt 16.049 3.171) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 16.049 3.171) (pt 16.049 -2.485) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 16.049 -2.485) (pt -4.619 -2.485) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -4.619 -2.485) (pt -4.619 3.171) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -4.369 2.921) (pt 15.799 2.921) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 15.799 2.921) (pt 15.799 -2.235) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 15.799 -2.235) (pt -4.369 -2.235) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -4.369 -2.235) (pt -4.369 2.921) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -4.369 1.016) (pt -2.464 2.921) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt 15.799 -2.235) (pt 15.799 2.921) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 15.799 2.921) (pt -4.369 2.921) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -4.369 2.921) (pt -4.369 0) (width 0.2))
		)
	)
	(symbolDef "CPC1998J" (originalName "CPC1998J")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinDes (text (pt 175 mils 25 mils) (rotation 0) (justify "Right") (textStyleRef "Normal"))) (pinName (text (pt 225 mils 0 mils) (rotation 0) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinDes (text (pt 175 mils -75 mils) (rotation 0) (justify "Right") (textStyleRef "Normal"))) (pinName (text (pt 225 mils -100 mils) (rotation 0) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 1300 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinDes (text (pt 1125 mils 25 mils) (rotation 0) (justify "Left") (textStyleRef "Normal"))) (pinName (text (pt 1075 mils 0 mils) (rotation 0) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 1300 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinDes (text (pt 1125 mils -75 mils) (rotation 0) (justify "Left") (textStyleRef "Normal"))) (pinName (text (pt 1075 mils -100 mils) (rotation 0) (justify "Right") (textStyleRef "Normal"))
		))

		(line (pt 200 mils 100 mils) (pt 1100 mils 100 mils) (width 8.0 mils))
		(line (pt 1100 mils 100 mils) (pt 1100 mils -200 mils) (width 8.0 mils))
		(line (pt 1100 mils -200 mils) (pt 200 mils -200 mils) (width 8.0 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 8.0 mils))

		(attr "RefDes" "RefDes" (pt 650 mils 150 mils) (justify center) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 650 mils -250 mils)  (justify center) (isVisible True) (textStyleRef "Normal"))

	)

	(compDef "CPC1998J" (originalName "CPC1998J") (compHeader (numPins 4) (numParts 1) (refDesPrefix K)
		)
		(compPin "1" (pinName "AC_LOAD(1)") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "AC_LOAD(2)") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "+LED") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "-LED") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "CPC1998J"))
		(attachedPattern (patternNum 1) (patternName "TO381P516X1991X2388-4P")
			(numPads 4)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
			)
		)
		(attr "Supplier_Name" "RS")
		(attr "RS Part Number" "")
		(attr "Manufacturer_Name" "IXYS SEMICONDUCTOR")
		(attr "Manufacturer_Part_Number" "CPC1998J")
		(attr "Allied_Number" "")
		(attr "Other Part Number" "")
		(attr "Description" "Solid State Relays - PCB Mount SP AC Solid State Power Switch")
		(attr "Datasheet Link" "http://www.ixysic.com/home/pdfs.nsf/www/CPC1998.pdf/$file/CPC1998.pdf")
		(attr "Height" "5.156")
		(attr "STEP Filename" "CPC1998J.stp")
	)

)