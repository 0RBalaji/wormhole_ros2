CREATE TABLE IF NOT EXISTS wormholes (

    id SERIAL PRIMARY KEY,                    -- Auto-incrementing ID
    map1_name VARCHAR(50) NOT NULL,           -- First map name
    map2_name VARCHAR(50) NOT NULL,           -- Second map name
    
    map1_position_x DOUBLE PRECISION NOT NULL,
    map1_position_y DOUBLE PRECISION NOT NULL,
    
    map1_orientation_z DOUBLE PRECISION DEFAULT 0,
    map1_orientation_w DOUBLE PRECISION DEFAULT 1,
    
    map2_position_x DOUBLE PRECISION NOT NULL,
    map2_position_y DOUBLE PRECISION NOT NULL,
    
    map2_orientation_z DOUBLE PRECISION DEFAULT 0,
    map2_orientation_w DOUBLE PRECISION DEFAULT 1,
    
    -- Timestamp for debugging
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);