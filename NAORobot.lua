function sysCall_init( ) 
    parent=sim.getObjectParent( sim.getObjectAssociatedWithScript( sim.handle_self ) )
    -- Get all the joints we wanna play back:
    allObjectsToExplore = { parent }
    allJointsOnTree = { }
    while ( #allObjectsToExplore > 0 ) do
        obj = allObjectsToExplore[1]
        table.remove( allObjectsToExplore, 1 )
        if ( sim.getObjectType( obj ) == sim.object_joint_type ) then
            table.insert( allJointsOnTree, obj )
        end
        index = 0
        while true do
            child = sim.getObjectChild( obj, index )
            if ( child == -1 ) then
                break
            end
            table.insert( allObjectsToExplore, child )
            index = index + 1
        end
    end
        
	recordedJointData = GetRecordedDataFromFile( sim.getScriptSimulationParameter( sim.handle_self, 'filePath' ) )
	jointNames = sim.getScriptSimulationParameter( sim.handle_self, 'jointNames' )
	jc = recordedJointData[1]
	jNames = { }
	l = 1
	for i = 1, jc, 1 do
		l2 = string.find( jointNames, "\0", l )
		table.insert( jNames, string.sub( jointNames, l, l2-1 ) )
		l = l2 + 1
	end
	jnts = { }
	for i = 1, #allJointsOnTree, 1 do
		h = allJointsOnTree[i]
		table.insert( jnts, allJointsOnTree[i] )
	end
	
	for i = 1, jc, 1 do
		for j = 1, #jnts, 1 do
			local h = jnts[j]
			local n = sim.getObjectName( h )
			local l = #n
			local l2 = string.find( n, '#' )
			if l2 then
				l = l2 - 1
			end
			n = string.sub( n, 1, l )
			if ( n == jNames[i] ) then
				allJointsOnTree[i] = jnts[j]
				break
			end
		end
	end

	initTime = sim.getSimulationTime()
end

function GetRecordedDataFromFile(path)
	local file = io.open( path, "r" )
	local recordedJointData = { }
	recordedJointData[1] = 40
	for line in file:lines( ) do
        recordedJointData[#recordedJointData + 1] = tonumber( line )
	end
	file:close()
	return recordedJointData
end

function sysCall_actuation() 

	-- Here we set one joint value for each joint in the model.
	-- The value is interpolated (since we might have different simulation step size settings now)
	if ( jc == #allJointsOnTree ) then
		local st = sim.getSimulationTime( ) - initTime
		-- search for two entry indices that come just before and just after the current simulation time (we will interpolate):
		local entrySize = jc + 1
		local totEntries = ( #recordedJointData - 1 ) / entrySize
		local b_i = -1
		local a_i = -1
		for i = 1, totEntries - 1, 1 do
			local t0 = recordedJointData[1 + ( i - 1 ) * entrySize + 1]
			local t1 = recordedJointData[1 + i * entrySize + 1]
			if ( t0 <= st ) and ( t1 > st ) then
				b_i = i-1
				a_i = i
				break
			end
		end

		
		if (b_i ~= -1) then
			local t0 = recordedJointData[1 + b_i * entrySize + 1]
			local t1 = recordedJointData[1 + a_i * entrySize + 1]
			local a = ( st - t0 ) / ( t1 - t0 )
			local a_ = 1 - a
			for i = 1, jc, 1 do
				local v0 = recordedJointData[1 + b_i * entrySize + 1 + i]
				local v1 = recordedJointData[1 + a_i * entrySize + 1 + i]
				local v01 = v0 * a_ + v1 * a
				local j = allJointsOnTree[i]
				m, o = sim.getJointMode( j )
				if ( m == sim.jointmode_force ) then
					r, w = sim.getObjectInt32Parameter( j, sim.jointintparam_ctrl_enabled )
					if ( w ~= 0) then
						sim.setJointTargetPosition( j, v01 ) -- we are in position control
					else
						sim.setJointTargetVelocity( j, v01 ) -- we are in velocity control or the motor is disabled
					end
				else
					sim.setJointPosition( j, v01 )
				end
			end
		else
			-- Let's restart with the same sequence:
			initTime = sim.getSimulationTime( )
			for i = 1, jc, 1 do
				recordedJointData[1 + 1 + i] = sim.getJointPosition( allJointsOnTree[i] )
			end
		end
	end
end 
